/******************************************************************************
    Copyright 2012 Mohamed Aslan

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

/*
	TODO: add support for more than one drive
*/

#include "dev/FDC.h"
#include "util/Log.h"
#include "mach/APIC.h"
#include "kern/AddressSpace.h"
#include "kern/Kernel.h"
#include "mach/platform.h"

bool FDC::init() {
	kcout << "Detecting available floppy disk controllers ...";
	// Read CMOS register 0x10
	out8(0x70, 0x10);
	// FIXME: a delay should be added before reading from CMOS port 0x71
	uint8_t cmos_fdc = in8(0x71);
	kcout << " [OK]\n";
	// Check for the master drive
	master_type = cmos_fdc >> 4;
	if(master_type) {
		kcout << "   * Master FD found. [" << fdc_type_name[master_type] << "]\n";
	}
	// Check for the slave drive
	slave_type = cmos_fdc & 0x0f;
	if(slave_type) {
		kcout << "   * Slave FD found. [" << fdc_type_name[master_type] << "]\n";
   }
	if(!master_type && !slave_type) {
		kcout << "   * No FD found in your system!\n";
		return false;
	}
	// Initialize the controller
   kcout << "   * Initializing the FDC ...";
   Machine::staticEnableIRQ(PIC::Floppy, 0x26);
   reset();
   kcout << " [OK]\n";
	// Which controller?
	send(CMD_VERSION);
	version = receive();
	switch(version) {
		case CHIP_NEC765:
			kcout << "      - NEC 765 controller found.\n";
			break;
		case CHIP_ENHANCED:
		case 0x81:	// from FreeBSD
			kcout << "      - Enhanced controller found.\n";
			break;
		default:
			kcout << "      - Unkown controller found.\n";
			break;
	}
	// If we have an enhanced control, configure it
	if(version == CHIP_ENHANCED) {
		// Configure our controller
		fifo_thres = 8;
		send(CMD_CONFIGURE);
		send(0x0);
		// enable implied seek & disable polling mode
		send((1 << CONFIG_EIS) | (1 << CONFIG_DPOLL) | ((fifo_thres - 1) & 0xf));
		send(0x0);	// use default precompensation
		// We do not need resets to mess with our configuration
		// Thus, issue lock command
		send(CMD_LOCK);
		if(!receive()) {
			// If successful should receive (1 << 4)
			kcerr << "FDC: error can not issue LOCK command.\n";
		}
	}
	// Calibrate i.e. move to cylinder 0
	if(!calibrate()) {
		kcerr << "FDC: calibration failed.\n";	
	}
	// Initialize DMA
	dmaInit();

   return true;
}

void FDC::staticInterruptHandler() {
	// Fired due to a FDC reset?
	if(fdc_reset) {
   	fdc_reset = false;
		return;
	}
}

bool FDC::read(uint16_t sector, uint8_t num, uint8_t *data) {
	CHS chs = lba2chs(sector);
	if(!seek(chs)) {
		kcerr << "FDC: seek failed.\n";
	}
	dmaSetup(num * 512);
	dmaRead();
	bool err = readSectors(chs);
	memcpy(data, fdcBuffer, num * 512);
	return err;
}

bool FDC::write(uint16_t sector, uint8_t num, uint8_t *data) {
	CHS chs = lba2chs(sector);
	if(!seek(chs)) {
		kcerr << "FDC: seek failed.\n";
	}
	memcpy(fdcBuffer, data, num * 512);
	dmaSetup(num * 512);
	dmaWrite();
	return writeSectors(chs);
}

bool FDC::readSectors(CHS chs) {
	// Turn on the motor
	turnMotorOn(true);
	// Try to read
	bool error = true;
	for(int i=0 ; i<3 ; i++) {
		send(CMD_READ | (1 << OPT_MT) | (1 << OPT_MF));
		send((chs.h << 2) | 0x0);	// Drive #0
		send(chs.c);	// cylinder
		send(chs.h);	// head
		send(chs.s);	// starting sector
		send(0x2);	// 512 per sector
		send(GEOM_SPT);	// sector count
		send(0x1b);	// GAP3 default size
		send(0xff); 
		// FIXME: delay
		uint8_t st0 = receive();
		uint8_t st1 = receive();
		uint8_t st2 = receive();
		uint8_t cyl = receive();
		uint8_t hd = receive();
		uint8_t sct = receive();
		uint8_t tmp = receive();
		// Update current location
		loc.c = cyl;
		loc.h = hd;
		loc.s = sct;
		// Check for errors
		// TODO: check other status registers for more errors
		if(st0 & 0xc0) {
			const char *status[] = { 0, "error", "invalid command", "drive not ready"};
			kcerr << "FDC: " << status[st0 >> 6] << "\n";
		}
		else {
			error = false;
			break;
		}
	}
	// Turn off the motor
	turnMotorOn(false);
	return error;
}

bool FDC::writeSectors(CHS chs) {
	// Turn on the motor
	turnMotorOn(true);
	// Try to write
	bool error = true;
	for(int i=0 ; i<3 ; i++) {
		send(CMD_WRITE | (1 << OPT_MT) | (1 << OPT_MF));
		send((chs.h << 2) | 0x0);	// Drive #0
		send(chs.c);	// cylinder
		send(chs.h);	// head
		send(chs.s);	// starting sector
		send(0x2);	// 512 per sector
		send(GEOM_SPT);	// sector count
		send(0x1b);	// GAP3 default size
		send(0xff); 
		// FIXME: delay
		uint8_t st0 = receive();
		uint8_t st1 = receive();
		uint8_t st2 = receive();
		uint8_t cyl = receive();
		uint8_t hd = receive();
		uint8_t sct = receive();
		uint8_t tmp = receive();
		// Update current location
		loc.c = cyl;
		loc.h = hd;
		loc.s = sct;
		// Check for errors
		// TODO: check other status registers for more errors
		if(st0 & 0xc0) {
			const char *status[] = { 0, "error", "invalid command", "drive not ready" };
			kcerr << "FDC: " << status[st0 >> 6] << "\n";
		}
		else {
			error = false;
			break;
		}
	}
	// Turn off the motor
	turnMotorOn(false);
	return error;
}

void FDC::reset() {
	// Reset & enable IRQ
   fdc_reset = true;
   out8(FDC_DOR, 0x0);	// reset
	// FIXME: a delay is required here  
   out8(FDC_DOR, (1 << NRESET) | (1 << IRQ)); // enable IRQ
	/*
	 	FIXME: at this moment, we should be waiting for an IRQ to be fired
		However, at this step KOS have not enabled interrupts yet!
		After being enabled, we may have a late one :)
	*/
	// Issue a specify command, the parameters' values are taken from OpenBSD
	send(CMD_SPECIFY);
	send(0xdf);	// SRT
	send(0x2);	// HLT
}

void FDC::turnMotorOn(bool turn_on) {
	// Should we turn it on?
	if(turn_on) {
		// Is it off?
		if(motor_on == false) {
			// Turn it on
			out8(FDC_DOR, (1 << MOTA) | (1 << IRQ) | (1 << NRESET));
			// FIXME: delay
			motor_on = true;
		}
	}
	else {
		// Is it on?
		if(motor_on == true) {
			// Turn it off
			out8(FDC_DOR, (1 << IRQ) | (1 << NRESET));
			// FIXME: delay
			motor_on = false;
		}
	}
}

bool FDC::calibrate() {
	// Turn motor on
	turnMotorOn(true);
	// Sometimes we can't reach cylinder 0 from the first calibration
	for(int i=0 ; i<5 ; i++) {
		// Issue a calibrate command
		send(CMD_CALIBRATE);
		send(0x0);	// Drive #0
		// FIXME: a long delay
		// Sense interrupt
		if(!sense()) {
			continue;
		}	
		// We reached cylinder 0?
		if(loc.c == 0x0) {
			// Turn off the motor
			turnMotorOn(false);
			return true;
		}
	}
	// Turn the motor off
	turnMotorOn(false);
	return false;
}

bool FDC::seek(CHS chs) {
	// Turn motor on
	turnMotorOn(true);
	// Sometimes we can't reach our cylinder the first trial
	for(int i=0 ; i<5 ; i++) {
		// Issue a seek command
		send(CMD_SEEK);
		send((chs.h << 2) | 0x0);	// Use drive #0
		send(chs.c);
		// FIXME: delay
		// Sense interrupt
		if(!sense()) {
			continue;
		}	
		// We reached our cylinder?
		if(loc.c == chs.c) {
			// Turn off the motor
			turnMotorOn(false);
			return true;
		}
	}
	// Turn the motor off
	turnMotorOn(false);
	return false;
}

// Should only be called after a calibration or a seek
bool FDC::sense() {
	// Sense interrupt
	send(CMD_SENSEI);
	uint8_t st0 = receive();
	uint8_t cur_cyl = receive();
	// Check for errors
	// TODO: check for other possible values (after reset)
	if(st0 == 0x20) {	// 0x20 is the value after successful calibration/seek
		loc.c = cur_cyl;
		return true;
	}
	else {
		return false;
	}
}

bool FDC::send(uint8_t b) {
	// Must check if FDC is ready before sending an commands
	for(int i=0 ; i<1000 ; i++) {
		// We are only ready if RQM is set & DIO is clear
		uint8_t msr = in8(FDC_MSR);
		if((msr & (1 << RQM)) && !(msr & (1 << DIO))) {
			out8(FDC_FIFO, b);
			return true;
		}
	}	
	return false;	
}

uint8_t FDC::receive() {
	// Must check if FDC is ready before sending any commands
	for(int i=0 ; i<1000 ; i++) {
		// We are only ready if RQM is set & DIO is clear
		uint8_t msr = in8(FDC_MSR);
		if((msr & (1 << RQM)) && (msr & (1 << DIO)) && (msr & (1 << CB))) {
			return in8(FDC_FIFO);
		}
	}	
	return -1;
}

FDC::CHS FDC::lba2chs(uint16_t lba) {
	CHS chs;
	chs.c = lba / (GEOM_SPT * GEOM_HPC);
	chs.h = (lba / GEOM_SPT) % GEOM_HPC;
	chs.s = (lba % GEOM_SPT) + 1;
	return chs;
}

// DMA
void FDC::dmaInit() {
	// FIXME: vaddr should be assigned and not assumed!
	kernelSpace.mapPages<1>(FDC_BUFFER, FDC_BUFFER, pagesize<1>(), true);
	fdcBuffer = (uint8_t *)FDC_BUFFER;
}

void FDC::dmaSetup(uint16_t count) {
	out8(0x0a, 0x06);	// mask DMA channel 2
	out8(0x0c, 0xff);	// reset master flip-flop
	out8(0x04, FDC_BUFFER & 0xff);	// address low 8-bits
	out8(0x04, FDC_BUFFER >> 0x8);	// address high 8-bits
	out8(0x81, 0x00);	// external page register
	out8(0x0c, 0xff);	// reset master flip-flop
	out8(0x05, (count - 1) & 0xff);	// (count - 1) low 8-bits
	out8(0x05, (count - 1) >> 0x8);	// (count - 1) high 8-bits
	out8(0x0a, 0x02);	// unmask DMA channel 2
}

void FDC::dmaRead() {
	out8(0x0a, 0x06);	// mask DMA channel 2
	out8(0x0b, 0x56);	// read
	out8(0x0a, 0x02);	// unmask DMA channel 2
}

void FDC::dmaWrite() {
	out8(0x0a, 0x06);	// mask DMA channel 2
	out8(0x0b, 0x5a);	// write
	out8(0x0a, 0x02);	// unmask DMA channel 2
}

