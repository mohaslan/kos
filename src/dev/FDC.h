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

#ifndef _FDC_h_
#define _FDC_h_

static const char* fdc_type_name[] = {
   "No Drive",
   "360 KB 5.25 Drive",
   "1.2 MB 5.25 Drive",
   "720 KB 3.5 Drive",
   "1.44 MB 3.5 Drive",
   "2.88 MB 3.5 drive"
};

// FDC IO Registers (0x3F0 -> 0x3F7) except 0x3F6
// Those only used, will be declared!
#define FDC_DOR 0x3F2   // Digital Output Register (R/W)
#define FDC_MSR 0x3F4   // Main Status Register (RO)
#define FDC_FIFO 0x3F5  // Data FIFO Register (R/W)

// DOR bits
// Those only used, will be declared!
#define MOTA  4
#define IRQ   3
#define NRESET 2
#define DSEL1 1
#define DSEL0 0

// MSR bits
// Those only used, will be declared!
#define RQM 7
#define DIO 6
#define CB 4

// Commands
// Those only used, will be declared!
#define CMD_SPECIFY 0x3
#define CMD_VERSION 0x10
#define CMD_CONFIGURE 0x13   // Enhanced controller
#define CMD_LOCK 0x94        // Enhanced controller
#define CMD_CALIBRATE 0x7
#define CMD_SENSEI 0x8
#define CMD_SEEK 0xf
#define CMD_READ 0x6
#define CMD_WRITE 0x5

// Controller chips
#define CHIP_NEC765 0x80
#define CHIP_ENHANCED 0x90

// CMD_CONFIGURE bits
#define CONFIG_EIS 6	// Enable implied seek
#define CONFIG_DFIFO 5	// Disable FIFO
#define CONFIG_DPOLL 4	// Disable polling mode

// CMD_READ & CMD_WRITE option bits
#define OPT_MT 7	// Multitrack
#define OPT_MF 6
#define OPT_SK 5

#define FDC_BUFFER 0x5000 // TODO: should be assigned by kernel
#define FDC_BUFFER_SIZE 32768 // 32KB, max 64 sectors

#define GEOM_SPT 18	// Sectors per track
#define GEOM_HPC 2 	// Heads per cylinder

#include "dev/Device.h"

class FDC : public Device {
	class CHS {
	public:
		uint8_t c, h, s;			
	};
private:
	uint8_t master_type = 0, slave_type = 0;
	volatile bool fdc_reset = false;
	uint8_t version = 0;
	uint8_t fifo_thres = 0;   // Only for enhanced controllers
	bool motor_on = false;
	CHS loc;
	uint8_t *fdcBuffer;
	void reset();
	bool calibrate();
	void turnMotorOn(bool);
	bool sense();
	bool seek(CHS);
	bool readSectors(CHS);
	bool writeSectors(CHS);
	bool send(uint8_t);
	uint8_t receive();
	CHS lba2chs(uint16_t);
	void dmaInit();
	void dmaSetup(uint16_t);
	void dmaRead();
	void dmaWrite();
public:
	FDC () { initAtBoot(); }
	virtual bool init();            __section(".kernel.init");
	void staticInterruptHandler();
	bool read(uint16_t, uint8_t, uint8_t *);
	bool write(uint16_t, uint8_t, uint8_t *);
};

#endif /* _FDC_h_ */
