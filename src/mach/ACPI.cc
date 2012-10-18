/******************************************************************************
    Copyright 2012 Martin Karsten

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
******************************************************************************/
extern "C" {
#include "extern/acpica/source/include/acpi.h"
}
#include "util/BlindHeap.h"
#include "util/Log.h"
#include "util/BlockingSync.h"
#include "mach/Machine.h"
#include "mach/Processor.h"
#include "mach/PCI.h"
#include "kern/AddressSpace.h"
#include "kern/Kernel.h"
#include "kern/KernelHeap.h"

#include <map>

static vaddr rsdp = 0;

// these lists keep track of memory allocations and mappings from acpica
static std::map<vaddr,mword,std::less<vaddr>,KernelAllocator<std::pair<vaddr,mword>>> allocations;
static std::map<vaddr,mword,std::less<vaddr>,KernelAllocator<std::pair<vaddr,mword>>> mappings;

static ACPI_OSD_HANDLER sciHandler = nullptr;
static void* sciContext = nullptr;

extern "C" void isr_handler_0x29() { // SCI interrupt
  if (sciHandler) {
    UINT32 retCode = sciHandler(sciContext);
    KASSERT(retCode == ACPI_INTERRUPT_HANDLED, retCode);
  }
  Processor::sendEOI();
}

void Machine::initACPI(vaddr r) {
  // set up information for acpica callback
  rsdp = r;

  // initialize acpi tables
  ACPI_STATUS status = AcpiInitializeTables( NULL, 0, true );
  KASSERT( status == AE_OK, status );

  // FADT is need to check for PS/2 keyboard
  acpi_table_fadt* fadt;
  char FADT[] = "FACP";
  status = AcpiGetTable( FADT, 0, (ACPI_TABLE_HEADER**)&fadt );
  KASSERT( status == AE_OK, status );

// the below fails on bochs & qemu -> only USB Legacy onboard?
//  KASSERT( fadt->BootFlags & ACPI_FADT_8042, "no PS/2 keyboard" );

  // MADT is needed for APICs, I/O-APICS, etc.
  acpi_table_madt* madt;
  char MADT[] = "APIC";
  status = AcpiGetTable( MADT, 0, (ACPI_TABLE_HEADER**)&madt );
  KASSERT( status == AE_OK, status );
  int madtLength = madt->Header.Length - sizeof(acpi_table_madt);
  vaddr apicPhysAddr = madt->Address;
  DBG::out(DBG::ACPI, "MADT: ", (ptr_t)apicPhysAddr, '/', madtLength);

  // disable 8259 PIC, if present
  if (madt->Flags & ACPI_MADT_PCAT_COMPAT) {
    DBG::out(DBG::ACPI, " - 8259");
    PIC::disable();
  }

  std::map<uint32_t,uint32_t,std::less<uint32_t>,KernelAllocator<std::pair<uint32_t,uint32_t>>> apicMap;
  std::map<uint32_t,IrqInfo,std::less<uint32_t>,KernelAllocator<std::pair<uint32_t,IrqInfo>>> irqMap;
  std::map<uint32_t,IrqOverrideInfo,std::less<uint32_t>,KernelAllocator<std::pair<uint32_t,IrqOverrideInfo>>> irqOverrideMap;

  // walk through subtables and gather information in dynamic maps
  acpi_subtable_header* subtable = (acpi_subtable_header*)(madt + 1);
  while (madtLength > 0) {
    KASSERT(subtable->Length <= madtLength, "ACPI MADT subtable error");
    switch (subtable->Type) {
    case ACPI_MADT_TYPE_LOCAL_APIC: {
      acpi_madt_local_apic* la = (acpi_madt_local_apic*)subtable;
      if (la->LapicFlags & 0x1) apicMap.insert( {la->ProcessorId, la->Id} );
    } break;
    case ACPI_MADT_TYPE_IO_APIC: {
      acpi_madt_io_apic* io = (acpi_madt_io_apic*)subtable;
      // map IO APIC into virtual address space
      kernelSpace.mapPages<1>(ioapicAddr, io->Address, pagesize<1>(), true);
      volatile IOAPIC* ioapic = (IOAPIC*)ioapicAddr;
      // get number of redirect entries, adjust irqCount
      uint8_t rdr = ioapic->getRedirects() + 1;
      if ( io->GlobalIrqBase + rdr > irqCount ) irqCount = io->GlobalIrqBase + rdr;
      // mask all interrupts and store irq/ioapic information
      for (uint8_t x = 0; x < rdr; x += 1 ) {
        ioapic->maskIRQ(x);
        irqMap.insert( {io->GlobalIrqBase + x, {io->Address, x}} );
      }
      // unmap IO APIC
      kernelSpace.unmapPages<1>(ioapicAddr, pagesize<1>(), true);
    } break;
    case ACPI_MADT_TYPE_INTERRUPT_OVERRIDE: {
      acpi_madt_interrupt_override* io = (acpi_madt_interrupt_override*)subtable;
      irqOverrideMap.insert( {io->SourceIrq, {io->GlobalIrq, io->IntiFlags}} );
    } break;
    case ACPI_MADT_TYPE_NMI_SOURCE:
      DBG::out(DBG::ACPI, " NMI_SOURCE"); break;
    case ACPI_MADT_TYPE_LOCAL_APIC_NMI: {
      DBG::out(DBG::ACPI, " LOCAL_APIC_NMI");
      acpi_madt_local_apic_nmi* ln = (acpi_madt_local_apic_nmi*)subtable;
      DBG::out(DBG::ACPI, '/', int(ln->ProcessorId), '/', int(ln->IntiFlags), '/', ln->Lint );
    } break;
    case ACPI_MADT_TYPE_LOCAL_APIC_OVERRIDE: {
      acpi_madt_local_apic_override* ao = (acpi_madt_local_apic_override*)subtable;
      apicPhysAddr = ao->Address;
      } break;
    case ACPI_MADT_TYPE_IO_SAPIC:
      DBG::out(DBG::ACPI, " IO_SAPIC"); break;
    case ACPI_MADT_TYPE_LOCAL_SAPIC:
      DBG::out(DBG::ACPI, " LOCAL_SAPIC"); break;
    case ACPI_MADT_TYPE_INTERRUPT_SOURCE:
      DBG::out(DBG::ACPI, " INTERRUPT_SOURCE"); break;
    case ACPI_MADT_TYPE_LOCAL_X2APIC:
      DBG::out(DBG::ACPI, " LOCAL_X2APIC"); break;
    case ACPI_MADT_TYPE_LOCAL_X2APIC_NMI:
      DBG::out(DBG::ACPI, " LOCAL_X2APIC_NMI"); break;
    case ACPI_MADT_TYPE_GENERIC_INTERRUPT:
      DBG::out(DBG::ACPI, " GENERIC_INTERRUPT"); break;
    case ACPI_MADT_TYPE_GENERIC_DISTRIBUTOR:
      DBG::out(DBG::ACPI, " GENERIC_DISTRIBUTOR"); break;
    default: KASSERT(false, "unknown ACPI MADT subtable");
    }
    madtLength -= subtable->Length;
    subtable = (acpi_subtable_header*)(((char*)subtable) + subtable->Length);
  }
  DBG::out(DBG::ACPI, kendl);

  KASSERT(apicMap.size() > 0, "no APIC found");

  // map APIC page and enable local APIC
  MSR::enableAPIC();                // should be enabled by default
  kernelSpace.mapPages<1>(lapicAddr, apicPhysAddr, pagesize<1>(), true);
  volatile LAPIC* lapic = (LAPIC*)lapicAddr;
  lapic->enable(0xf8);               // confirm spurious vector at 0xf8

  // determine bspApicID, cpuCount, bspIndex, and create processorTable
  bspApicID = lapic->getLAPIC_ID();
  cpuCount = apicMap.size();
  processorTable = new Processor[cpuCount];
  int idx = 0;
  for (const std::pair<uint32_t,uint32_t>& ap : apicMap) {
    processorTable[idx].init(ap.second, ap.first, *Processor::getFrameManager());
    if (ap.second == bspApicID) bspIndex = idx;
    idx += 1;
  }
  DBG::outln(DBG::Basic, "CPUs: ", cpuCount, '/', bspIndex, '/', bspApicID);

  // IOAPIC irq range should at leave cover standard PIC irqs
  KASSERT(irqCount >= PIC::Max, irqCount );

  // fill irqTable
  irqTable = new IrqInfo[irqCount];
  for (uint32_t i = 0; i < irqCount; i += 1) {
    irqTable[i] = { 0, 0 };
  }
  for (const std::pair<uint32_t,IrqInfo>& i : irqMap) {
    KASSERT( i.first < irqCount, i.first );
    irqTable[i.first] = i.second;
  }

  // fill irqOverrideTable
  irqOverrideTable = new IrqOverrideInfo[irqCount];
  for (uint32_t i = 0; i < irqCount; i += 1) {
    irqOverrideTable[i] = { i, 0 };
  }
  for (const std::pair<uint32_t,IrqOverrideInfo>& i : irqOverrideMap) {
    KASSERT( i.first < irqCount, i.first )
    irqOverrideTable[i.first] = i.second;
  }
}

static ACPI_STATUS display(ACPI_HANDLE handle, UINT32 level, void* ctx, void** retval) {
  ACPI_STATUS status;
  ACPI_DEVICE_INFO* info;
  ACPI_BUFFER path;
  char buffer[256];

  path.Length = sizeof(buffer);
  path.Pointer = buffer;
  status = AcpiGetName(handle, ACPI_FULL_PATHNAME, &path);
  KASSERT( status == AE_OK, status );
  status = AcpiGetObjectInfo(handle, &info);
  KASSERT( status == AE_OK, status );
#ifdef ACPICA_OUTPUT
  char* name = (char*)&info->Name;
  AcpiOsPrintf("%c%c%c%c HID: %s, ADR: %.8X, Status: %x\n",
    name[0], name[1], name[2], name[3],
    info->HardwareId.String, info->Address, info->CurrentStatus);
#endif
  return AE_OK;
}

void Machine::parseACPI() {
  ACPI_STATUS status;

  status = AcpiInitializeSubsystem();
  KASSERT( status == AE_OK, status );
//  status = AcpiReallocateRootTable();
//  KASSERT( status == AE_OK, status );  
  status = AcpiLoadTables();
  KASSERT( status == AE_OK, status );

  // TODO: install notification handlers

  status = AcpiEnableSubsystem(ACPI_FULL_INITIALIZATION);
  KASSERT( status == AE_OK, status );
  status = AcpiInitializeObjects(ACPI_FULL_INITIALIZATION);
  KASSERT( status == AE_OK, status );

  ACPI_HANDLE sysBusHandle = ACPI_ROOT_OBJECT;
  AcpiWalkNamespace(ACPI_TYPE_DEVICE, sysBusHandle, uint32_limit, display, NULL, NULL, NULL);

  status = AcpiTerminate();
  KASSERT( status == AE_OK, status );

  // clean up ACPI leftover memory allocations and mappings
  for (std::pair<vaddr,mword> a : allocations ) {
    AcpiOsFree( ptr_t(a.first) );
  }
  for (std::pair<vaddr,mword> m : mappings ) {
    AcpiOsUnmapMemory( ptr_t(m.first), m.second );
  }
}

ACPI_STATUS AcpiOsInitialize(void) { return AE_OK; }

ACPI_STATUS AcpiOsTerminate(void) { return AE_OK; }

ACPI_PHYSICAL_ADDRESS AcpiOsGetRootPointer(void) {
  return rsdp;
}

ACPI_STATUS AcpiOsPredefinedOverride(const ACPI_PREDEFINED_NAMES* InitVal,
  ACPI_STRING* NewVal) {
  *NewVal = nullptr;
  return AE_OK;
}

ACPI_STATUS AcpiOsTableOverride(ACPI_TABLE_HEADER* ExistingTable,
  ACPI_TABLE_HEADER** NewTable) {
  *NewTable = nullptr;
  return AE_OK;
}

ACPI_STATUS AcpiOsPhysicalTableOverride(ACPI_TABLE_HEADER* ExistingTable,
  ACPI_PHYSICAL_ADDRESS* NewAddress, UINT32* NewTableLength) {
  *NewAddress = 0;
  *NewTableLength = 0;
  return AE_OK;
}

ACPI_STATUS AcpiOsCreateLock(ACPI_SPINLOCK* OutHandle) {
  *OutHandle = new SpinLock;
  return AE_OK;
}

void AcpiOsDeleteLock(ACPI_SPINLOCK Handle) {
  delete reinterpret_cast<SpinLock*>(Handle);
}

ACPI_CPU_FLAGS AcpiOsAcquireLock(ACPI_SPINLOCK Handle) {
  reinterpret_cast<SpinLock*>(Handle)->acquire();
  return 0;
}

void AcpiOsReleaseLock(ACPI_SPINLOCK Handle, ACPI_CPU_FLAGS Flags) {
  reinterpret_cast<SpinLock*>(Handle)->release();
}

ACPI_STATUS AcpiOsCreateSemaphore(UINT32 MaxUnits, UINT32 InitialUnits,
  ACPI_SEMAPHORE* OutHandle) {
  *OutHandle = new Semaphore(InitialUnits);
  return AE_OK;
}

ACPI_STATUS AcpiOsDeleteSemaphore(ACPI_SEMAPHORE Handle) {
  delete reinterpret_cast<Semaphore*>(Handle);
  return AE_OK;
}

ACPI_STATUS AcpiOsWaitSemaphore(ACPI_SEMAPHORE Handle, UINT32 Units,
  UINT16 Timeout) {

  KASSERT(Timeout == 0xFFFF || Timeout == 0, Timeout);
  for (UINT32 x = 0; x < Units; x += 1) reinterpret_cast<Semaphore*>(Handle)->P();
  return AE_OK;
}

ACPI_STATUS AcpiOsSignalSemaphore(ACPI_SEMAPHORE Handle, UINT32 Units) {
  for (UINT32 x = 0; x < Units; x += 1) reinterpret_cast<Semaphore*>(Handle)->V();
  return AE_OK;
}

#if 0
ACPI_STATUS AcpiOsCreateMutex(ACPI_MUTEX* OutHandle) {
  KASSERT(false,"");
  return AE_ERROR;
}

void AcpiOsDeleteMutex(ACPI_MUTEX Handle) { KASSERT(false,""); }

ACPI_STATUS AcpiOsAcquireMutex(ACPI_MUTEX Handle, UINT16 Timeout) {
  KASSERT(false,"");
  return AE_ERROR;
}

void AcpiOsReleaseMutex(ACPI_MUTEX Handle) { KASSERT(false,""); }
#endif

void* AcpiOsAllocate(ACPI_SIZE Size) {
  void* ptr = (void*)BlindHeap<KernelHeap>::alloc(Size);
  DBG::outln(DBG::ACPI, "acpi alloc: ", ptr, '/', Size);
  memset(ptr, 0, Size); // zero out memory to play it safe...
  allocations.insert( {vaddr(ptr), Size} );
  return ptr;
}

void AcpiOsFree(void* Memory) {
  DBG::outln(DBG::ACPI, "acpi free: ", Memory);
  BlindHeap<KernelHeap>::release(vaddr(Memory));
  allocations.erase( vaddr(Memory) );
}

void* AcpiOsMapMemory(ACPI_PHYSICAL_ADDRESS Where, ACPI_SIZE Length) {
  DBG::outln(DBG::ACPI, "acpi map: ", (ptr_t)Where, '/', (ptr_t)Length);
  laddr lma = align_down(Where, pagesize<1>());
  kernelSpace.mapPages<1>(lma, lma, align_up(Length + Where - lma, pagesize<1>()), false);
  mappings.insert( {lma,Length} );
  return (void*)Where;
}

void AcpiOsUnmapMemory(void* LogicalAddress, ACPI_SIZE Size) {
  DBG::outln(DBG::ACPI, "acpi unmap: ", LogicalAddress, '/', (ptr_t)Size);
  vaddr vma = align_down(vaddr(LogicalAddress), pagesize<1>());
  kernelSpace.unmapPages<1>(vma, align_up(Size + vaddr(LogicalAddress) - vma, pagesize<1>()), false);
  mappings.erase(vma);
}

ACPI_STATUS AcpiOsGetPhysicalAddress(void* LogicalAddress,
  ACPI_PHYSICAL_ADDRESS* PhysicalAddress) {

  *PhysicalAddress = PageManager::vtol<1>(vaddr(LogicalAddress));
  return AE_OK;
}

ACPI_STATUS AcpiOsCreateCache(char* CacheName, UINT16 ObjectSize,
  UINT16 MaxDepth, ACPI_CACHE_T** ReturnCache) {
  *ReturnCache = (ACPI_CACHE_T*)KernelHeap::createCache(ObjectSize);
  return AE_OK;
}

ACPI_STATUS AcpiOsDeleteCache(ACPI_CACHE_T* Cache) {
  KernelHeap::destroyCache(reinterpret_cast<KernelHeap::Cache*>(Cache));
  return AE_OK;
}

ACPI_STATUS AcpiOsPurgeCache(ACPI_CACHE_T* Cache) {
  reinterpret_cast<KernelHeap::Cache*>(Cache)->purge();
  return AE_OK;
}

void* AcpiOsAcquireObject(ACPI_CACHE_T* Cache) {
  vaddr addr = reinterpret_cast<KernelHeap::Cache*>(Cache)->alloc();
  if (addr == illaddr) return nullptr;
  memset((void*)addr, 0, reinterpret_cast<KernelHeap::Cache*>(Cache)->getSize());
  return (void*)addr;
}

ACPI_STATUS AcpiOsReleaseObject(ACPI_CACHE_T* Cache, void* Object) {
  reinterpret_cast<KernelHeap::Cache*>(Cache)->release((vaddr)Object);
  return AE_OK;
}

ACPI_STATUS AcpiOsInstallInterruptHandler(UINT32 InterruptNumber,
  ACPI_OSD_HANDLER ServiceRoutine, void* Context) {

  if (InterruptNumber != 9 || !ServiceRoutine) return AE_BAD_PARAMETER;
  if (sciHandler) return AE_ALREADY_EXISTS;
  Processor::disableInterrupts();
  sciHandler = ServiceRoutine;
  sciContext = Context;
  Processor::enableInterrupts();
  return AE_OK;
}

ACPI_STATUS AcpiOsRemoveInterruptHandler(UINT32 InterruptNumber,
  ACPI_OSD_HANDLER ServiceRoutine) {

  if (!sciHandler) return AE_NOT_EXIST;
  if (InterruptNumber != 9 || !ServiceRoutine || ServiceRoutine != sciHandler) {
    return AE_BAD_PARAMETER;
  }
  Processor::disableInterrupts();
  sciHandler = nullptr;
  sciContext = nullptr;
  Processor::enableInterrupts();
  return AE_OK;
}

ACPI_THREAD_ID AcpiOsGetThreadId(void) {
  return (ACPI_THREAD_ID)Processor::getThread();
}

ACPI_STATUS AcpiOsExecute(ACPI_EXECUTE_TYPE Type,
  ACPI_OSD_EXEC_CALLBACK Function, void* Context) {

  KASSERT(false,"");
  return AE_ERROR;
}

void AcpiOsWaitEventsComplete(void* Context) { KASSERT(false,""); }

void AcpiOsSleep(UINT64 Milliseconds) { KASSERT(false,""); }

void AcpiOsStall(UINT32 Microseconds) { KASSERT(false,""); }

void AcpiOsWaitEventsComplete() { KASSERT(false,""); }

ACPI_STATUS AcpiOsReadPort(ACPI_IO_ADDRESS Address, UINT32* Value, UINT32 Width) {
  switch (Width) {
    case 8: *Value = in8(Address); break;
    case 16: *Value = in16(Address); break;
    case 32: *Value = in32(Address); break;
    default: return AE_ERROR;
  }
  return AE_OK;
}

ACPI_STATUS AcpiOsWritePort(ACPI_IO_ADDRESS Address, UINT32 Value, UINT32 Width) {
  switch (Width) {
    case 8: out8(Address,Value); break;
    case 16: out16(Address,Value); break;
    case 32: out32(Address,Value); break;
    default: return AE_ERROR;
  }
  return AE_OK;
}

ACPI_STATUS AcpiOsReadMemory(ACPI_PHYSICAL_ADDRESS Address, UINT64* Value, UINT32 Width) {
  KASSERT(false,"");
  return AE_ERROR;
}

ACPI_STATUS AcpiOsWriteMemory(ACPI_PHYSICAL_ADDRESS Address, UINT64 Value, UINT32 Width) {
  KASSERT(false,"");
  return AE_ERROR;
}

ACPI_STATUS AcpiOsReadPciConfiguration( ACPI_PCI_ID* PciId, UINT32 Reg,
  UINT64* Value, UINT32 Width) {

  // PciId->Segment ?
  *Value = PCI::readConfigWord(PciId->Bus, PciId->Device, PciId->Function, Reg, Width);
  return AE_OK;
}

ACPI_STATUS AcpiOsWritePciConfiguration(ACPI_PCI_ID* PciId, UINT32 Reg,
  UINT64 Value, UINT32 Width) {

  // PciId->Segment ?
  PCI::writeConfigWord(PciId->Bus, PciId->Device, PciId->Function, Reg, Width, Value);
  return AE_OK;
}

BOOLEAN AcpiOsReadable(void* Pointer, ACPI_SIZE Length) {
  KASSERT(false,"");
  return false;
}

BOOLEAN AcpiOsWritable(void* Pointer, ACPI_SIZE Length) {
  KASSERT(false,"");
  return false;
}

UINT64 AcpiOsGetTimer(void) {
  KASSERT(false,"");
  return 0;
}

ACPI_STATUS AcpiOsSignal(UINT32 Function, void* Info) {
  KASSERT(false,"");
  return AE_ERROR;
}

void ACPI_INTERNAL_VAR_XFACE AcpiOsPrintf(const char* Format, ...) {
#ifdef ACPICA_OUTPUT
  va_list args;
  va_start(args, Format);
  kcdbg << Printf(Format, args) << kendl;
#endif
}

void AcpiOsVprintf(const char* Format, va_list Args) {
#ifdef ACPICA_OUTPUT
  kcdbg << Printf(Format, args) << kendl;
#endif
}

void AcpiOsRedirectOutput(void* Destination) { KASSERT(false,""); }

ACPI_STATUS AcpiOsGetLine(char* Buffer, UINT32 BufferLength, UINT32* BytesRead) {
  KASSERT(false,"");
  return AE_ERROR;
}

void* AcpiOsOpenDirectory(char* Pathname, char* WildcardSpec, char RequestedFileType) {
  KASSERT(false,"");
  return nullptr;
}

char* AcpiOsGetNextFilename(void* DirHandle) {
  KASSERT(false,"");
  return nullptr;
}

void AcpiOsCloseDirectory(void* DirHandle) { KASSERT(false,""); }
