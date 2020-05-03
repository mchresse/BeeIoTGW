// BSD 3-Clause License
// 
// BeeIoT Weight Scale project using an own BeeIoTWAN based on LoRa Radio functions
// using a SX1276 LoRa modem chip (e.g. Dragino LoRa/GPS HAT) connected to a 
// Raspberry Pi platform.
//	
// Copyright (c) 2019, Randolph Esser
// All rights reserved.
// 
// This file provides incorporated routines from hal.c + hal.h layer of the project:
//	Hardware Abstraction Layer (HAL) targeted to Raspberry Pi and 
//	Dragino LoRa/GPS HAT
// implemented by
// 
// Copyright (c) 2017, Wolfgang Klenk
// All rights reserved.
/*
 * This file provides incorporated routines out of the great MCCI-LMIC project 
 *		from radio.c + lmic.h files 
 * as implemented by
 * Copyright (c) 2014-2016 IBM Corporation.
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of the <organization> nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <cstdint>
#include <cstring>
#include <iostream>
#include <sys/time.h>
#include <time.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <errno.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>

#include "BeeIoTWan.h"
#include "beeiot.h"
#include "gwqueue.h"	// using STL container classes
#include "regslora.h"
#include "beelora.h"
#include "radio.h"

//******************************************************************************
// global runtime Variables:

// LogFlags: from main.cpp
extern unsigned int		lflags;	// BeeIoT log flag field

// Ptr. to global Lora Modem Configuration table (in NwSrv.cpp)
extern gwbind_t		gwtab;	// GateWay related config sets e.g. for isr_init()

//******************************************************************************
// local/static runtime Variables:

// Assign ISR of lora instance to global IRQ table
static void isr_init (modemcfg_t * mod);

// IRQ Mask flags of RX(Single/Contiguous) or TX Mode
// in case of FreqHopOn -> add IRQ_LORA_FHSSCH_MASK here and check in ISR2
static const u1_t rxlorairqmask[] = {
		[RXMODE_SINGLE] = IRQ_LORA_RXDONE_MASK|IRQ_LORA_RXTOUT_MASK,
		[RXMODE_SCAN]   = IRQ_LORA_RXDONE_MASK,
		[RXMODE_RSSI]   = 0x00,
};

static const char * rxloraOMstring[] = {
		[OPMODE_SLEEP]	= "SLEEP",
		[OPMODE_STANDBY]= "STANDBY",
		[OPMODE_FSTX]	= "FSTX",
		[OPMODE_TX]		= "TX",
		[OPMODE_FSRX]	= "FSRX",
		[OPMODE_RX]		= "RXCONT",
		[OPMODE_RX_SINGLE]="RXSINGLE",
		[OPMODE_CAD]	= "CAD",
};

	// structure must fit to enum sf_t in BeeIoTWAN.h
static const u2_t LORA_RXDONE_FIXUP[] = {
		[FSK]  =     us2osticks(0), // (   0 ticks)
		[SF7]  =     us2osticks(0), // (   0 ticks)
		[SF8]  =  us2osticks(1648), // (  54 ticks)
		[SF9]  =  us2osticks(3265), // ( 107 ticks)
		[SF10] =  us2osticks(7049), // ( 231 ticks)
		[SF11] = us2osticks(13641), // ( 447 ticks)
		[SF12] = us2osticks(31189), // (1022 ticks)
};
static const u2_t LORA_TXDONE_FIXUP = us2osticks(43);

//******************************************************************************
// Radio Constructor:
// Init LoRa Modem by BeeIoTWAN Defaults: (on SX1276 chip only)
// - Reset Modem chip and read version type (SX1276 = 0x12)
// - Set OpMode SLEEP -> needed to change any cfg.
// - Set frequency REG_FRF_MSB.._LSB (0x06-0x08) <= 'FREQ' (typ.: EU868_F1)
// - Set LoRa SyncWord -> for BIoTWAN: use 0x12, (official LoRaWan takes 0x34)
// - MC1: 0x72: BW125 + CR4/5 + Explicite header Mode  e.g. with BW125SF7: 6kbit/s
// - MC2: 0x74: SF7 + No-RXCont + CRC-ON + RX_TOut_MSB = 0 
// - MC3: 0x04: LowDataRate depending on SF11/12 and BW125/250 + AGC Auto On
// - Set SymbTimeout: SF7 -> 8 symbols
// - Set Preamble Length: 0x0008
// - Set MAX_PAYLOAD_LENGTH = 0x80	-> BeeIoT-WAN Limit; half FIFO buffer
// - Set PayLoadLength		= 0x40	=> will be redfined by RX/TX task
// - Set REG_HOP_PERIOD		= 0x00 -> disabled; no automatic frequ. hoping=> no FHSS Mask
// - Set REG_FIFO_ADDR_PTR	= 0x00 -> Reset RX buffer to FIFO base
// - Set REG_LNA: LOW NOISE AMPLIFIER to maximum (0x21) => will be redfined by RX/TX task
// - Set RegPaRamp to 50usec =0x08
// - Set REG_DIO_MAPPING_1	to DIO0_RX_CONT == RXDone	=> will be redfined by RX/TX task
// - Set IRQ Flags Mask =0xFF -> Disable all IRQs by now. => redefined by TX/TX task
// - Assign ISR to IRQ mapping table and activate ISR by ISR-level Semaphore=0
//******************************************************************************
Radio::Radio(gwbind_t & gwtab, int mid) : gwt(gwtab) {
	modemp = &gwtab.gwset[mid];		// get modem settings for this new instance
	gwt.modem[mid] = this;			// save new Modem Instance Ptr. to Instance Index in global Binding table
	mset.modemid = modemp->modemid;	// save mid per instance locally (should be identical as mid)

	if(modemp->modemid != mid){	// should never happen !
		BHLOG(LOGLORAW) printf("  SetupLora(): gwset-ModemID:%i differs from mid: %i, (using %i)\n",
				(int)modemp->modemid, mid, (int)mset.modemid);
		throw (int) EX_RADIO_INIT;		// Exc.ID: RADIO Init failed -> break;
		// return;
	}
	
	this->dioISR = NULL;				// init empty ISR jumptable ptr. (not used yet)
	this->mset.gwq = NULL;

	setchannelconfig(modemp->chncfgid);	// Get modem channel cfg set "JOIN default" from user ini file
	chncfg_t *ccfg = &mset.chncfg;		// temp. ptr. to new intialized local modem channel cfg. set for further modem init
	
	// Reset all Channel config parameters to BeeIoT-WAN defaults:
	BHLOG(LOGLORAR) printf("  SetupLora(): Reset SX device of Modem%d\n", (int)modemp->modemid);

	// Send Reset to assumed SPI:RFM96 module (== SX127x base) using GPIO-RST line
	resetModem();

	if( getchiptype() <= 0 ){	// expect supported chiptype  > 0
		BHLOG(LOGLORAW) printf("  SetupLora(): no supported chiptype 0x%02X of Modem%i detected\nExit() -> Check GPIO# in config.ini\n", 
				(unsigned char)mset.chiptype, (int)mset.modemid);
		throw (int) EX_RADIO_INIT;		// Exc.ID: RADIO Init failed -> break;
		// return;
	}

	// Now we have a supported LoRa modem chip at SPI port:
	// Set RFM96 Module to LoRa&sleep mode for further configuration
	setLoraMode();				// Set LoRa Mode -> already results in SLEEP Mode
    setopmode(OPMODE_SLEEP);	// Do it official to keep 'currentMode' also up to date

	configChannelFreq();		// set Channel Frequency by local modem cfg. set
	configLoraModem();			// Preset Reg_Config 1-3

	// Preset LoRaSW: but will be reassigned at rxlora() & txlora() again (!)
    writeReg(LORARegSyncWord, LoRaSW); // Preset LoRa sync word -> see BIoTWAN.h
	
	// set Preamble pattern length for radio pkg detection
	writeReg(LORARegPreambleMsb, 0x00);
	writeReg(LORARegPreambleLsb, 0x08);	

	// Now Calculate and set symbol timeout depending on spreading factor: 
	// -> at least preamble field must be accepted
    if (ccfg->sf == SF10 || ccfg->sf == SF11 || ccfg->sf == SF12) {
        writeReg(LORARegSymbTimeoutLsb,RX_TOUT_LSB);	// RX_TOut_LSB = 5 x Tsymbol
    } else {
		// give 8 symbols time for preamble detection
        writeReg(LORARegSymbTimeoutLsb,0x08);	// RX_TOut_LSB = 8 x Tsymbol
    }

	// PreSet the maximum possible downlink payload size to default: 64 bytes
	// Payload greater than this threshold will not be demodulated. 
	// Receiver will immediately go back to “stand-by” low power mode
	// => Will be redefined during RX/TX sessions dynamically
	writeReg(LORARegPayloadMaxLength, MAX_PAYLOAD_LENGTH);	// def. 64 Byte TX data
    writeReg(LORARegPayloadLength, 0x40);			// typ. 64 Byte

	// Set Frequ.-Hopping Mode:
	//    writeReg(LORARegHopPeriod,0xFF);	// set Freq.Hopping period to max. time -> 255 symbols
    writeReg(LORARegHopPeriod,0x00);		// No implicite Frequ. Hopping supported
	// RegPllHop(0x44) remains unchanged (default)

	// Reset FiFo: Set RD/WR Ptr to start of RX field base
    writeReg(LORARegFifoAddrPtr, readReg(LORARegFifoRxBaseAddr));
    writeReg(RegLna, LNA_RX_GAIN);  // Max LNA gain: G1 + DefLNACurr + NoBoost
	writeReg(RegPaRamp, (readReg(RegPaRamp) & 0xF0) | 0x08); // set PA ramp-up time 50 uSec

	// On Cont. RX Mode: DIO0-IRQ gets active if SyncAddress bits are valid
	// ISR routine have to wait for 'Payload ready' before reading Payload data !
	// ToDO: Check out in ISR routine if wait is really needed
    writeReg(RegDioMapping1, MAP_DIO0_LORA_RXDONE|MAP_DIO1_LORA_NOP|MAP_DIO2_LORA_NOP);
	writeReg(LORARegIrqFlagsMask, 0xFF);

	//	PrintLoraStatus(LOGALL);
	/* LoRa Modem Register: Example Output at this point
	---------------------------------------------------------------
	LoRa Modem Register Status:     Version: 0x12  LoRa Modem OpMode : 0x80
	  MODEM_CONFIG1     : 0x72  DIO 0-3 Mapping   : 0x3C  SYNC_WORD         : 0x12
	  MODEM_CONFIG2     : 0x74  DIO 4-5 Mapping   : 0x00  PREAMBLE_LENGTH   : 0x0008
	  MODEM_CONFIG3     : 0x04  LNA GAIN          : 0x20  PA_CONFIG         : 0x4F
	  PLL BitRateFrac   : 0xD0  HOP_PERIOD        : 0x00  PA_RAMP           : 0x08  
	  LoRa Modem Status : 0x10  FIFO_RX_BASE_ADDR : 0x00  IRQ_FLAGS_MASK    : 0xFF
	  FIFO_RX_CURR_ADDR : 0x00  FIFO_TX_BASE_ADDR : 0x80  IRQ_FLAGS         : 0x00
	  FIFO_ADDR_PTR     : 0x00  FIFO_LAST_BYTE_WR : 0x00  RX_NB_BYTES       : 0x00
	  MAX_PAYLOAD_LENGTH: 0x80  Last PACKET_SNR   : 0x00  #of valid Headers : 0x0000
	  PAYLOAD_LENGTH    : 0x40  Last PACKET_RSSI  : 0x00  #of valid Packets : 0x0000
	  Current_RSSI      : 0x00  IRQ Level         : 0  
	*/
	// setup Radio layer house keeping data 
	// + get AES encoded Random bit field for random()
	radio_init();

	// Define and activate ISR handling:
	// - Mask all IRQs but RxDone for RXCONT Mode
	//   We us RXCont Mode: Do we need RX_Header as well ? RXDone should be enough
	//   in RXonly mode we would have to add: IRQ_LORA_HEADER_MASK
	writeReg(LORARegIrqFlagsMask, ~(IRQ_LORA_RXDONE_MASK|IRQ_LORA_CRCERR_MASK)); // for RXDone

	// ToDo: this table is for future use: not used by isr_init() by now!
	dioISR = new IrqHandler[3];	// set mapping table of DIO0..2 ISR function ptr
	dioISR[0] = &Radio::MyIRQ0;
	dioISR[1] = &Radio::MyIRQ1;
	dioISR[2] = &Radio::MyIRQ2;
	
	TXDoneFlag=1;	// Init TX Flag: TXDone=1 -> LoRa channel is free

	// Assign ISR to GPIO line
	// => Radio member function using implicit C-function call: 
	//    by Lambda Frame conversion
	isr_init(modemp);	// assign ISRs to IRQ Port <modem> referenced by this
	mset.irqlevel =0;	// Activate ISRs: enable ISR Semaphore

	BHLOG(LOGLORAR) printf("  SetupLora(): Done\n");
	return;
}

//******************************************************************************
// destructor RADIO class
// Release Semtech SX chip and SPI channel
Radio::~Radio(){
	// Stop IRQs: further ISR handling blocked
	mset.irqlevel = 1;
    this->hal_disableIRQs();

	// Reset RFM96 module == SX1276 base
	this->resetModem();

	// Set RFM96 Module to defined LoRa-sleep mode
	this->setLoraMode();

	// done implicit
	if(this->dioISR)	// has it already been created ?(should not, if exception at check SXtypes)
		delete[] this->dioISR;	// release ISR jumptable
}

//*****************************************************************************
// GetChipType()
// Detect Lora Modem chip by version id
// Known versions:
//	0x22: SX1272	not supported
//	0x12: SX1276	supported
// Return: 
//	version		supported (!) chiptype version
//				see above: as delivered by Read regVersion cmd.
//	 0			known but not supported
//	-1			unknown chip type detected
// global mset.chiptype	gets read chiptype (anyway)
int Radio::getchiptype(void){
    mset.chiptype = readReg(RegVersion);	// read chitype directly from HW(SPI port)
	int rc;
	
	switch(mset.chiptype){
	case 0x22:	// sx1272
		BHLOG(LOGLORAR) printf("  SetupLora(): SX1272 detected\n");
		rc = 0;					// known but not supported (yet)
	case 0x12:	// sx1276
		BHLOG(LOGLORAR) printf("  SetupLora(): SX1276 detected\n");
		rc = (int)mset.chiptype;// known & supported
		break;
	default:
		rc = -1;				// not supported
		break;
	}
	return (rc);
}



//*****************************************************************************
// Update/Init Modem local Channel configuration set
// index used for global cfgchntab[cfgidx] access
//
void Radio::setchannelconfig(byte cfgidx){
chncfg_t * ccfg = &mset.chncfg;	// ptr to local modem cfg set for modem runtime
	
	if(cfgidx > MAX_CHANNELS-1){ // unknown cfg. channel index
		cfgidx = 0;				 // unknown channel cfg -> set to JOIN default
	}
	// get user settings of Channel cfg by cfgidx: 
	ccfg->freq		= cfgchntab[cfgidx].frq;	// =EU868_F1..9,DN (EU868_F1: 868.1MHz)
	ccfg->pw		= cfgchntab[cfgidx].pwr;	// =2-16  TX PA Mode (14)
	ccfg->sf		= cfgchntab[cfgidx].sfbegin;// =0..8 Spreading factor FSK,7..12,SFrFu (1:SF7)
	ccfg->bw		= cfgchntab[cfgidx].band;	// =0..3 RFU Bandwidth 125-500 (0:125kHz)
	ccfg->cr		= cfgchntab[cfgidx].cr;		// =0..3 Coding mode 4/5..4/8 (0:4/5)

	// static settings from BIoTWAN.h (used as BIoT API default for GW & Node)
	ccfg->ih		= IHDMODE;				// =1 implicit header mode
	ccfg->ihlen		= IHEADERLEN;			// =0..n if IH -> header length (0)
	ccfg->nocrc		= NOCRC;				// =0/1 no CRC check used for Pkg (0)
	ccfg->noRXIQinv = NORXIQINV;			// =0/1 flag to switch RX+TX IQinv. on/off (1)
	return;	
}

//*****************************************************************************
// SX RADIO layer HAL functions
//*****************************************************************************
void Radio::hal_pin_nss (u1_t val) {
    digitalWrite(modemp->iopins.sxcs, val);
}

// perform 8-bit SPI transaction with radio
// write given byte out to radio, read byte from radio and return value.
u1_t Radio::hal_spi (u1_t out) {
    u1_t res = wiringPiSPIDataRW(0, &out, 1);
	if (res <0){
		fprintf(stderr, "HAL: Cannot send data on SPI: %s\n", strerror(errno));
		// exit ?)
	}
    return out;
}

// SPI transfer with address and one single byte.
u1_t Radio::hal_spi_single (u1_t address, u1_t out) {
   u1_t buffer[2];
   buffer[0] = address;
   buffer[1] = out;

   u1_t rc = wiringPiSPIDataRW(0, buffer, 2);
   if (rc < 0) {
      fprintf(stderr, "HAL: Cannot send data on SPI: %s\n", strerror(errno));
      // exit ?
   }
   return buffer[1];
}

// Reset RFM96 module == SX1276 base
void Radio::resetModem(void){
	digitalWrite(modemp->iopins.sxrst, HIGH);
	delay(100);
	digitalWrite(modemp->iopins.sxrst, LOW);
	delay(100);		// wait >100us till Reset was recognized and processed
	digitalWrite(modemp->iopins.sxrst, HIGH);
	delay(100);		// wait >5ms till chip is ready
}

void Radio::writeReg (u1_t addr, u1_t data ) {
    hal_pin_nss(0);
    hal_spi(addr | 0x80);
    hal_spi(data);
    hal_pin_nss(1);
}

u1_t Radio::readReg (u1_t addr) {
    hal_pin_nss(0);
    hal_spi(addr & 0x7F);
    u1_t val = hal_spi(0x00);
    hal_pin_nss(1);
    return val;
}

void Radio::writeBuf (u1_t addr, xref2u1_t buf, u1_t len) {
    hal_pin_nss(0);
    hal_spi(addr | 0x80);
    for (u1_t i=0; i<len; i++) {
        hal_spi(buf[i]);
    }
    hal_pin_nss(1);
}

void Radio::readBuf (u1_t addr, xref2u1_t buf, u1_t len) {
    hal_pin_nss(0);
    hal_spi(addr & 0x7F);
    for (u1_t i=0; i<len; i++) {
        buf[i] = hal_spi(0x00);
    }
    hal_pin_nss(1);
}

void Radio::setopmode (u1_t mode) {
	mode &= OPMODE_MASK|OPMODE_LORA;
	mset.currentMode = (readReg(RegOpMode) & ~OPMODE_MASK) | mode;
    writeReg(RegOpMode, mset.currentMode);	// inkl. last LF mode status
	BHLOG(LOGLORAR) printf("    Radio: New OpMode: %s (0x%02X)\n", 
			rxloraOMstring[mode&OPMODE_MASK], (unsigned int)mset.currentMode);
	mset.currentMode &= (OPMODE_LORA | OPMODE_MASK);		// Filter LF Mode
}

byte Radio::getopmode(void){
	return(mset.currentMode);
}
int Radio::getmodemidx(void){	// deliver index of modem instance
	return(this->mset.modemid);
}
long Radio::getchannelfrq(void){
	return(mset.chncfg.freq);
}
sf_t Radio::getspreading(void){
	return(mset.chncfg.sf);
}
bw_t Radio::getband(void){
	return(mset.chncfg.bw);
}
cr_t Radio::getcoding(void){
	return(mset.chncfg.cr);
}
int Radio::getirqlevel(void){
	return(mset.irqlevel);
}

void Radio::setLoraMode(void) {
    // def:LoRa with LF Mode - for SX1276 only !
	mset.currentMode = OPMODE_LORA | OPMODE_SLEEP; // Start in SLeep Mode +
	writeReg(RegOpMode, mset.currentMode);		  // HF Mode, no shared FSK Reg Access
    BHLOG(LOGLORAR) printf("    Radio: Set LoRa-Mode & Sleep\n");
}


// configure LoRa modem (cfg1, cfg2)
void Radio::configLoraModem (void) {
    u1_t mc1 = 0, mc2 = 0, mc3 = 0;
	int sbw; 
	const char *scr;
	chncfg_t * ccfg = &mset.chncfg;
	
    // set ModemConfig1
        switch (ccfg->bw) {
        case BW125: mc1 |= SX1276_MC1_BW_125; sbw=125; break;
        case BW250: mc1 |= SX1276_MC1_BW_250; sbw=250; break;
        case BW500: mc1 |= SX1276_MC1_BW_500; sbw=500; break;
        default:		//  ASSERT(0);
			BHLOG(LOGLORAR) printf("    Radio: configLoraModem:  Warning_wrong BW setting: %i kHz\n", sbw);
        }
        switch( ccfg->cr ) {
        case CR_4_5: mc1 |= SX1276_MC1_CR_4_5; scr = "4/5"; break;
        case CR_4_6: mc1 |= SX1276_MC1_CR_4_6; scr = "4/6"; break;
        case CR_4_7: mc1 |= SX1276_MC1_CR_4_7; scr = "4/7"; break;
        case CR_4_8: mc1 |= SX1276_MC1_CR_4_8; scr = "4/8"; break;
		default:		//  ASSERT(0);
			BHLOG(LOGLORAR) printf("    Radio: configLoraModem:  Warning_unknown CR value: %i\n",(int) scr);
        }

        if (ccfg->ih) {
            mc1 |= SX1276_MC1_IMPLICIT_HEADER_MODE_ON;
            writeReg(LORARegPayloadLength, ccfg->ihlen); // required length
        }
        writeReg(LORARegModemConfig1, mc1);

    // set ModemConfig2 (rxsymto typ. 0x0)
        mc2 = (SX1272_MC2_SF7 + ((ccfg->sf-1)<<4) /* + ((rxsymto >> 8) & 0x3)*/ );
        if (!ccfg->nocrc) { 
            mc2 |= SX1276_MC2_RX_PAYLOAD_CRCON;
        }
        writeReg(LORARegModemConfig2, mc2);
        
    // set ModemConfig3
        mc3 = SX1276_MC3_AGCAUTO;
        if( ((ccfg->bw == BW125) && (ccfg->sf == SF11 || ccfg->sf == SF12)) ||
			 (ccfg->bw == BW250) && (ccfg->sf == SF12)){
            mc3 |= SX1276_MC3_LOW_DATA_RATE_OPTIMIZE;
        }
        writeReg(LORARegModemConfig3, mc3);

    // Errata 2.1: Sensitivity optimization with 500 kHz bandwidth
        u1_t rHighBwOptimize1;
        u1_t rHighBwOptimize2;

        rHighBwOptimize1 = 0x03;
        rHighBwOptimize2 = 0;

        if (ccfg->bw == BW500) {
            if (ccfg->freq > SX127X_FREQ_LF_MAX) { // e.g. for 868MHz
                rHighBwOptimize1 = 0x02;	
                rHighBwOptimize2 = 0x64;
            } else {
                rHighBwOptimize1 = 0x02;
                rHighBwOptimize2 = 0x7F;
            }
        }

        writeReg(LORARegHighBwOptimize1, rHighBwOptimize1);
        if (rHighBwOptimize2 != 0)
            writeReg(LORARegHighBwOptimize2, rHighBwOptimize2);
		
		BHLOG(LOGLORAR) printf("    ConfigLoRa: mc1=0x%02X, mc2=0x%02X, mc3=0x%02X, Channel= %.6lf MHz\n",
				(unsigned char)mc1, (unsigned char)mc2, (unsigned char)mc3, (double)ccfg->freq/1000000);
} // end configLoraModem()

void Radio::configChannelFreq (void) {
chncfg_t * ccfg = &mset.chncfg;
    // set frequency: FQ = (FRF * 32 Mhz) / (2 ^ 19)
    u8_t frf = ((u8_t)ccfg->freq << 19) / 32000000;
    writeReg(RegFrfMsb, (u1_t)(frf>>16));
    writeReg(RegFrfMid, (u1_t)(frf>> 8));
    writeReg(RegFrfLsb, (u1_t)(frf>> 0));
//	printf("    ConfigChannel= %.6lf MHz\n", (double)ccfg->freq/1000000);
}


//******************************************************************************
// ConfigPower()
// On the SX1276, we have several possible configs.
// 1) using RFO, MaxPower==0: in that case power is -4 to 11 dBm
// 2) using RFO, MaxPower==7: in that case, power is 0 to 14 dBm
//      (can't select 15 dBm).
//	note we can use -4..11 w/o Max and then 12..14 w/Max, and
//	we really don't need to ask anybody.
// 3) using PA_BOOST, PaDac = 4: in that case power range is 2 to 17 dBm;
//	use this for 15..17 if authorized.
// 4) using PA_BOOST, PaDac = 7, OutputPower=0xF: in that case, power is 20 dBm
//		(and perhaps 0xE is 19, 0xD is 18 dBm, but datasheet isn't clear.)
//    and duty cycle must be <= 1%.
//
// In addition, there are some boards for which PA_BOOST can only be used if the
// channel frequency is greater than SX127X_FREQ_LF_MAX.
//
// The SX1272 is similar but has no MaxPower bit:
// 1) using RFO: power is -1 to 13 dBm (datasheet implies max OutputPower value is 14 for 13 dBm)
// 2) using PA_BOOST, PaDac = 0x84: power is 2 to 17 dBm;
//	use this for 14..17 if authorized
// 3) using PA_BOOST, PaDac = 0x87, OutputPower = 0xF: power is 20dBm
//    and duty cycle must be <= 1%
//
// The general policy is to use the lowest power variant that will get us where we
// need to be.
//

void Radio::configPower (void) {
chncfg_t *ccfg = &mset.chncfg;
    // our input parameter -- might be different than pw!
    s1_t const req_pw = (s1_t)ccfg->pw;
    s1_t eff_pw;	// the effective power
    u1_t policy;	// the policy; we're going to compute this.
    u1_t rPaConfig; // what we'll write to RegPaConfig
    u1_t rPaDac;	// what we'll write to RegPaDac
    u1_t rOcp;		// what we'll write to RegOcp

	// negotiate power class
    if (req_pw >= 20) {
        policy = LMICHAL_radio_tx_power_policy_20dBm;
        eff_pw = 20;
    } else if (req_pw >= 14) {
        policy = LMICHAL_radio_tx_power_policy_paboost;
        if (req_pw > 17) {
            eff_pw = 17;
        } else {
            eff_pw = req_pw;
        }
    } else { // default e.g. for pw<14
        policy = LMICHAL_radio_tx_power_policy_rfo;
        if (req_pw < -4) {
            eff_pw = -4;
        } else {
            eff_pw = req_pw;
        }
    }

//	ask for forgiveness: but not here
//    policy = hal_getTxPowerPolicy(policy, eff_pw, freq);

    switch (policy) {
    default:
    case LMICHAL_radio_tx_power_policy_rfo:
        rPaDac = SX127X_PADAC_POWER_NORMAL;
        rOcp = SX127X_OCP_MAtoBITS(80);

        if (eff_pw > 14)
            eff_pw = 14;
        if (eff_pw > 11) {
            // some Semtech code uses this down to eff_pw == 0.
            rPaConfig = eff_pw | SX1276_PAC_MAX_POWER_MASK;
        } else {
            if (eff_pw < -4)
                eff_pw = -4;
            rPaConfig = eff_pw + 4;
        }
        break;

    // some radios (HopeRF RFM95W) don't support RFO well,
    // so the policy might *raise* rfo to paboost. That means
    // we have to re-check eff_pw, which might be too small.
    // (And, of course, it might also be too large.)
    case LMICHAL_radio_tx_power_policy_paboost:
        // It seems that SX127x doesn't like eff_pw 10 when in FSK mode.
        if (ccfg->sf == FSK && eff_pw < 11) {
            eff_pw = 11;
        }
        rPaDac = SX127X_PADAC_POWER_NORMAL;
        rOcp = SX127X_OCP_MAtoBITS(100);
        if (eff_pw > 17)
            eff_pw = 17;
        else if (eff_pw < 2)
            eff_pw = 2;
        rPaConfig = (eff_pw - 2) | SX1276_PAC_PA_SELECT_PA_BOOST;
        break;

    case LMICHAL_radio_tx_power_policy_20dBm:
        rPaDac = SX127X_PADAC_POWER_20dBm;
        rOcp = SX127X_OCP_MAtoBITS(130);
        rPaConfig = 0xF | SX1276_PAC_PA_SELECT_PA_BOOST;
        break;
    }


    writeReg(RegPaConfig, rPaConfig);
    writeReg(RegPaDac, (readReg(RegPaDac) & ~SX127X_PADAC_POWER_MASK) | rPaDac);
    writeReg(RegOcp, rOcp | SX127X_OCP_ENA);
}

// get random seed from wideband noise rssi
void Radio::radio_init(void) {
	// initial init: no IRQ request allowed
	mset.irqlevel = 0;
    hal_disableIRQs();
	
// set the tcxo input, if needed ???
//    if (hal_queryUsingTcxo())
//        writeReg(RegTcxo, readReg(RegTcxo) | RegTcxo_TcxoInputOn);

     // seed 15-byte randomness via noise rssi
    rxlora(RXMODE_RSSI,0);
	BHLOG(LOGLORAR) printf("  radio_init(): get random field values:\n");
    while( (readReg(RegOpMode) & OPMODE_MASK) != OPMODE_RX ); // continuous rx ?
    for(int i=1; i<16; i++) {
		BHLOG(LOGLORAR) printf("  %2d:", i);
        for(int j=0; j<8; j++) {
            u1_t b; // wait for two non-identical subsequent least-significant bits
            while( (b = readReg(LORARegRssiWideband) & 0x01) == (readReg(LORARegRssiWideband) & 0x01) );
            mset.randbuf[i] = (mset.randbuf[i] << 1) | b;
			BHLOG(LOGLORAR) printf(" %0.2X", (unsigned char)mset.randbuf[i]);
        }
		BHLOG(LOGLORAR) printf("\n");
    }
    mset.randbuf[0] = 16; // set initial index
  
    setopmode(OPMODE_SLEEP);
	writeReg(LORARegIrqFlagsMask, 0xFF);

	// recover cfg. from RSSI read
	configLoraModem();			// Preset Reg_Config 1-3

    hal_enableIRQs();
	BHLOG(LOGLORAW) printf("  radio_init(): SX dev. detected and reset, randbuf[] initialized\n");
}

// return next random byte derived from seed buffer
// (buf[0] holds index of next byte to be returned)
u1_t Radio::radio_rand1 (void) {
    u1_t i = mset.randbuf[0];
    if( i==16 ) {
        os_aes(AES_ENC, (xref2u1_t)mset.randbuf, 16); // encrypt seed with any key
        i = 0;
    }
    u1_t v = mset.randbuf[i++];
    mset.randbuf[0] = i;
    return v;
}

u1_t Radio::LoRa_random(void){
  return readReg(LORARegRssiWideband);
}

u1_t Radio::radio_rssi (void) {
    hal_disableIRQs();
    u1_t r = readReg(LORARegRssiValue);
    hal_enableIRQs();
    return r;
}






//*************************************************************
// PrintLoraStatus()
// Dumps most importent SX127x registers in LoRa Mode.
// requires sleep mode !
// input: logtype
//		LOGDYN	Dumps all dynamic registers related to current RX/TX action
//		LOGSTAT	Dumps only static configuration settings
//      LOGALL	Dumps all Register as availabel in current mode
//*************************************************************
void Radio::PrintLoraStatus(int logtype){
	printf("---------------------------------------------------------------\n");
	printf("LoRa Modem Register Status:     Version: 0x%2X", readReg(RegVersion));

	if(logtype == LOGSTAT || logtype == LOGALL){
		printf("  LoRa Modem OpMode : 0x%02X\n",readReg(RegOpMode));

		printf("  MODEM_CONFIG1     : 0x%02X",	readReg(LORARegModemConfig1));
		printf("  DIO 0-3 Mapping   : 0x%02X",	readReg(RegDioMapping1));
		printf("  SYNC_WORD         : 0x%02X\n",readReg(LORARegSyncWord));

		printf("  MODEM_CONFIG2     : 0x%02X",	readReg(LORARegModemConfig2));
		printf("  DIO 4-5 Mapping   : 0x%02X",	readReg(RegDioMapping2));
		printf("  PREAMBLE_LENGTH   : 0x%02X%02X\n", readReg(LORARegPreambleMsb), readReg(LORARegPreambleLsb));

		printf("  MODEM_CONFIG3     : 0x%02X",	readReg(LORARegModemConfig3));
		printf("  LNA GAIN          : 0x%02X",	readReg(RegLna));
		printf("  PA_CONFIG         : 0x%02X\n",readReg(RegPaConfig));

		printf("  PLL BitRateFrac   : 0x%02X",	readReg(RegBitRateFrac));
		printf("  HOP_PERIOD        : 0x%02X",	readReg(LORARegHopPeriod));
		printf("  PA_RAMP           : 0x%02X",  readReg(RegPaRamp));
		printf("  \n");
	}

	if(logtype == LOGDYN || logtype == LOGALL){
		if(logtype != LOGALL){
			printf("  LoRa Modem OpMode : 0x%02X\n",readReg(RegOpMode));
		}
		printf("  LoRa Modem Status : 0x%02X",	readReg(LORARegModemStat));
		printf("  FIFO_RX_BASE_ADDR : 0x%02X",	readReg(LORARegFifoRxBaseAddr));
		printf("  IRQ_FLAGS_MASK    : 0x%02X\n",readReg(LORARegIrqFlagsMask));

		printf("  FIFO_RX_CURR_ADDR : 0x%02X",	readReg(LORARegFifoRxCurrentAddr));
		printf("  FIFO_TX_BASE_ADDR : 0x%02X",	readReg(LORARegFifoTxBaseAddr));
		printf("  IRQ_FLAGS         : 0x%02X\n",readReg(LORARegIrqFlags));

		printf("  FIFO_ADDR_PTR     : 0x%02X",	readReg(LORARegFifoAddrPtr));
		printf("  FIFO_LAST_BYTE_WR : 0x%02X",	readReg(LORARegFifoRxByteAddr));
		printf("  RX_NB_BYTES       : 0x%02X\n",readReg(LORARegRxNbBytes));

		printf("  MAX_PAYLOAD_LENGTH: 0x%02X",	readReg(LORARegPayloadMaxLength));
		printf("  Last PACKET_SNR   : 0x%02X",	readReg(LORARegPktSnrValue));
		printf("  #of valid Headers : 0x%02X%02X\n",readReg(LORARegRxHeaderCntValueMsb), readReg(LORARegRxHeaderCntValueLsb));

		printf("  PAYLOAD_LENGTH    : 0x%02X",	readReg(LORARegPayloadLength));
		printf("  Last PACKET_RSSI  : 0x%02X",	readReg(LORARegPktRssiValue));
		printf("  #of valid Packets : 0x%02X%02X\n", readReg(LORARegRxPacketCntValueMsb), readReg(LORARegRxpacketCntValueLsb));

		printf("  Current_RSSI      : 0x%02X",	readReg(LORARegRssiValue));
		printf("  IRQ Level         : %i", mset.irqlevel);
		printf("  \n");
	}

	printf("\n");
	return;
} // end of LogLoRaStatus()



void Radio::txlora (byte* frame, byte dataLen) {
	chncfg_t * ccfg = &mset.chncfg;

    // select LoRa modem (from sleep mode)
    setLoraMode();
	if((readReg(RegOpMode) & OPMODE_LORA) != OPMODE_LORA){
		BHLOG(LOGLORAW) printf("    txlora: TX in FSK Mode !\n");
		return;
	}
    // enter standby mode (required for FIFO loading))
    setopmode(OPMODE_STANDBY);

    // configure LoRa modem/channel for TX (requires Standby/Sleep Mode)
    configLoraModem();	// Set Modem Cfg. registers 1,2,3
    configChannelFreq();	// configure Channel frequency
	
	writeReg(RegPaRamp, 0x08);     // set PA ramp-up time 50 uSec, clear FSK bits
    configPower();		// configure Antenna Output power

	// set sync word
    writeReg(LORARegSyncWord, LoRaSW);
    
    // set the IRQ mapping DIO0=TxDone DIO1=NOP DIO2=NOP
    writeReg(RegDioMapping1, MAP_DIO0_LORA_TXDONE|MAP_DIO1_LORA_NOP|MAP_DIO2_LORA_NOP);

    // clear all radio IRQ flags
    writeReg(LORARegIrqFlags, 0xFF);
    // mask all IRQs but TxDone
    writeReg(LORARegIrqFlagsMask, ~IRQ_LORA_TXDONE_MASK);

    // initialize the payload size and address pointers    
    writeReg(LORARegFifoTxBaseAddr, 0x00);
    writeReg(LORARegFifoAddrPtr, 0x00);
    writeReg(LORARegPayloadLength, dataLen);
       
    // download buffer to the radio FIFO
    writeBuf(RegFifo, frame, dataLen);

    // enable antenna switch for TX
	// Nothing to do. There is no such pin in the Lora/GPS HAT module.
	//    digitalWrite(pins.rxtx, val);
	//  or   hal_pin_rxtx(1);
    
    // now we actually start the transmission
    setopmode(OPMODE_TX);
	
	gettimeofday(&mset.now, NULL);
    mset.txstart = (uint32_t)(mset.now.tv_sec*1000000 + mset.now.tv_usec); // get TimeStamp in seconds
	BHLOG(LOGLORAR) printf("    txlora(): TX pkg sent (len=0x%02X) -> TX Mode entered\n", (unsigned char) dataLen);
	BHLOG(LOGLORAR) printf("    <%ul> TXMODE, freq=%ul, len=%d, SF=%d, BW=%d, CR=4/%d, IH=%d\n",
            (unsigned long) mset.txstart, (unsigned long) ccfg->freq, (unsigned int)dataLen, (unsigned int)ccfg->sf+6,
			(unsigned char) ccfg->bw == BW125 ? 125 : (ccfg->bw == BW250 ? 250 : 500),
			(unsigned char) ccfg->cr == CR_4_5 ? 5 : (ccfg->cr == CR_4_6 ? 6 : (ccfg->cr == CR_4_7 ? 7 : 8)),
			(unsigned int)  ccfg->ih);
}

//******************************************************************************
// StartTX()
// start transmition
// if async=0: return after TXDoneFlag set by ISR; =0 return immediately
int Radio::starttx (byte* frame, byte dataLen, bool async) {
	if(!frame || !dataLen)
		return(-1);	// wrong input params
	
	// Start TX for LoRa modem only
	TXDoneFlag = 0;	// Spawn TXDoneFlag -> Set to 1 by ISR if TXDone IRQ occurs
	txlora(frame, dataLen);
	
	if (async) {// TX async: return immediately
		// but spend some grace time for the radio to be save for next SPI access
		delayMicroseconds(150);		// wait for  a while till SX is stable
	} else {	// TX sync: lets wait till TXDone-ISR reports TX completion
		int count = 0;
		// the ISR will inform us about completion by TXDoneFlag = 1;
		while (!TXDoneFlag){	// Poll TX compl. flag processed by ISR routine
			// wait for max. # of sent bytes with Tsymb=1ms (SF7)
			delay(dataLen+8);	// Give ISR also some time to validate TXDone
								// min.: length of HD+payload+preamble Bytes
			// ToDO: timeout action if TX fails in a certain time -> retry ?
			if(++count == MAXTXTO){
				BHLOG(LOGLORAR) printf("    Radio: TX Time Out !\n");
				setopmode(OPMODE_SLEEP);	// reset Modem FIFO and go sleeping
				return(-2);	// report TX TimeOut
			}
		}
		BHLOG(LOGLORAR) printf("    Radio: TX Done (TO_count=%i) !\n", count);
	}
    // the radio will go automatically to STANDBY mode as soon as TX has been finished
	return(0);	// TX Data at least initiated
}

//***********************************************************
// starts LoRa receiver (time=LMIC.rxtime, timeout=LMIC.rxsyms, 
//		 				result=LMIC.frame[LMIC.dataLen])
void Radio::rxlora (u1_t rxmode, int rxto) {
chncfg_t *ccfg = &mset.chncfg;

	// rxto : wait time in sec. in rx single mode
    // select LoRa modem (from sleep mode)

    setLoraMode();
	if((readReg(RegOpMode) & OPMODE_LORA) != OPMODE_LORA){
		BHLOG(LOGLORAW) printf("    rxlora: RX in FSK Mode !\n");
		return;
	}
    // enter standby mode (warm up))
    setopmode(OPMODE_STANDBY);    // don't use MAC settings at startup

    if(rxmode == RXMODE_RSSI) { // use fixed settings for rssi scan
		BHLOG(LOGLORAR) printf("    rxlora: Set RXMODE_RSSI in mc1+mc2\n");
        writeReg(LORARegModemConfig1, RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG1);
        writeReg(LORARegModemConfig2, RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG2);
    } else { // Cfg. for single or continuous rx mode
	    // configure LoRa modem/channel for TX (requires Standby/Sleep Mode)
	    configLoraModem();		// Set Modem Cfg. registers 1,2,3
	    configChannelFreq();	// configure Channel frequency
    }
    // set LNA gain
    writeReg(RegLna, LNA_RX_GAIN); 
    // set max payload size
    writeReg(LORARegPayloadMaxLength, MAX_PAYLOAD_LENGTH);
//    writeReg(LORARegPayloadLength, 0x40);		// LoRaWAN limit: 64 Byte fix
	
    // use inverted I/Q signal (prevent mote-to-mote communication)
	// new with v1.6 XXX: use flag to switch on/off inversion
    if (ccfg->noRXIQinv) {
        writeReg(LORARegInvertIQ, readReg(LORARegInvertIQ) & ~(1<<6));
    } else {
        writeReg(LORARegInvertIQ, readReg(LORARegInvertIQ)|(1<<6));
    }	

	// Errata 2.3 - receiver spurious reception of a LoRa signal
    u1_t const rDetectOptimize = (readReg(LORARegDetectOptimize) & 0x78) | 0x03;
    if (ccfg->bw < BW500) {
        writeReg(LORARegDetectOptimize, rDetectOptimize);
        writeReg(LORARegIffReq1, 0x40);
        writeReg(LORARegIffReq2, 0x40);
    } else {
        writeReg(LORARegDetectOptimize, rDetectOptimize | 0x80);
    }
	
    // set symbol timeout (for single rx)
    writeReg(LORARegSymbTimeoutLsb, RX_TOUT_LSB);
    // set sync word
    writeReg(LORARegSyncWord, LoRaSW);
    // configure DIO mapping DIO0=RxDone DIO1=RxTout DIO2=NOP (orig)
	// on Cont. RX Mode: DIO0 Active if SyncAddress bits are valid
	// IRQ routine have to wait for Payload ready before reading Payload data !
    writeReg(RegDioMapping1, MAP_DIO0_LORA_RXDONE|MAP_DIO1_LORA_RXTOUT|MAP_DIO2_LORA_NOP);

//	Freq Hopping not supported yet
//	writeReg(LORARegHopPeriod, 0xFF);

    // clear all radio IRQ flags
    writeReg(LORARegIrqFlags, 0xFF);
    // enable required radio IRQs
    writeReg(LORARegIrqFlagsMask, ~rxlorairqmask[rxmode]);

	// Nothing to do. There is no such pin in the Lora/GPS HAT module.
    // enable antenna switch for RX -> not needed here
	//     hal_pin_rxtx(0);

	// Use FIFO from the buttom up: 0...n (255)
    writeReg(LORARegFifoAddrPtr, 0);
    writeReg(LORARegFifoRxBaseAddr, 0);

	gettimeofday(&mset.now, NULL);
    uint32_t tstamp = (uint32_t)(mset.now.tv_sec*1000000 + mset.now.tv_usec); // get TimeStamp in seconds
	BHLOG(LOGLORAR) printf("    <%ul> RXMODE:%d, freq=%ul,SF=%d, BW=%d, CR=4/%d, IH=%d\n",
			(unsigned long)tstamp, (unsigned int)rxmode, (unsigned long)ccfg->freq, (unsigned char)ccfg->sf+6,
			(unsigned char)ccfg->bw == BW125 ? 125 : (ccfg->bw == BW250 ? 250 : 500),
			(unsigned char)ccfg->cr == CR_4_5 ? 5 : (ccfg->cr == CR_4_6 ? 6 : (ccfg->cr == CR_4_7 ? 7 : 8)), 
			(unsigned char)ccfg->ih);

    // now instruct the radio to receive new pkg
    if (rxmode == RXMODE_SINGLE) { // one shot with TimeOUT
//        hal_waitUntil(LMIC.rxtime); // busy wait until exact rx time window ends
        setopmode(OPMODE_RX_SINGLE);
		delay(rxto*1000);			// wait in sec for incoming data
		BHLOG(LOGLORAR) printf("    rxlora(): STart RX in SINGLE Mode for %i sec.)\n", rxto);
    } else if (rxmode == RXMODE_SCAN){ // continous rx (scan or rssi)
        setopmode(OPMODE_RX); 
		BHLOG(LOGLORAR) printf("    rxlora(): Start RX in CONT Mode\n");
    } else{ // must be RXMODE_RSSI
        setopmode(OPMODE_RX); 
		BHLOG(LOGLORAR) printf("    rxlora(): RSSI SCAN initiated\n");		
	}
	mset.rxtime = 0;	// will be init by ISR at RXDONE
}

void Radio::startrx (u1_t rxmode, int rxtime) {
	// ASSERT( (readReg(RegOpMode) & OPMODE_MASK) == OPMODE_SLEEP );
	// basically not needed: rx routine forces STDBY Mode

	// LoRa modem only !
        rxlora(rxmode, rxtime);
    // the radio will go back to STANDBY mode as soon as the RX is finished
    // or timed out, and the corresponding IRQ will inform us about completion.
}



//******************************************************************************
// myradio_irq_handler()
// Radio ISR called by HAL extension IRQ handler MyIRQ0-2 
// assigned to corresponding Semtech SX1276 DIO0-2 lines.
// Each IRQ line triggers myradio_irq_handler() bypassing the DIO line number:
//
// 0. Check LoRa Mode	-> BIOT use only LoRa Modem type
//	1. Check TXDone		-> set BeeIotTXFlag flag only
//	2. Check RX Queue full -> no RX-Queue buffer left => shortcut ISR, no action
//	3. Check RXDone		
//		4. CRC Check	-> shortcut ISR, no action
//		5. a)Get RSSI & SNR Status and b) evaluate SNR threshold
//		    -> if < SNR Threshold => shortcut ISR
//		6. Check Package size: must be in range of BeeIoT WAN protocol specification 
//		7. RD Radio Queue -> fill RX Queue buffer + incr. BeeIotTXFlag Semaphore
//	8. RXTOUT-Check	  -> shortcut ISR, no action (BIoT uses RXCont Mode only)
//	9. Acknowledge all IRQ flags at once
//10. FSK Mode IRQ => should never happen ! -> shortcut ISR, no action
//11. Set OPMode to Sleep Mode -> OM polled by BIoT Log Service
//
// INPUT:		
//		DIO	number of initiating IRQ line {0,1,2}={DIO0,DIO1,DIO2}
//
// OUTPUT(used global/public):
//	currentMode			Radio OP Mode of SX127x chip (Idle, Sleep, RX, TX, ...)
//	BeeIotTXFlag		TX Semaphor for startTX() service routine; =1: TX Done
//	txend				number of System ticks used for TX process
//	LORA_TXDONE_FIXUP	txend  correction value for duty time calculation by spreading factor SF
//	BeeIotRXFlag		RX Semaphor for startRX() service routine; >=1: RX Done
//	rxtime				time stamp of RX Done status (system clock)
//	LORA_RXDONE_FIXUP	rxtime correction value for duty time calculation by spreading factor SF
//	bw					Bandwidth of currently used RX channel (for exact RX timestamp calculation)
//	RXPkgIsrIdx			current IN-Queue Write Index to Pkg-element
//	MyRXData[]			IN-Queue of BIoT RX-Service routine
//
// RETURN:
// In all cases ISR ends up in setopmode(OPMODE_SLEEP) 
// -> OPMOde polled by BIoT Logger Service to setup always a new RXCont Mode cycle 
// incl. SX1776 RX configuration. If a valid package was identified -> MyRXData[]-Element
// is filled up and BeeIoTRXFlag+=1 (Queue-Semaphore = fill level)

void Radio::myradio_irq_handler (byte dio) {
unsigned long tstamp;
struct timeval now;
byte flags;
byte mode;
chncfg_t * ccfg = &mset.chncfg;

// save current timestamp
	gettimeofday(&now, NULL);	
	mode = readReg(RegOpMode);	// get current Radio OpMode
	
	// Workaround: Spurious missing LoRa OPMode flag ... just wait some ms and read it again
    if( (mode & OPMODE_LORA) == 0) { // FSK Mode ? (not expected)
		tstamp = (unsigned long)(now.tv_sec*1000000 + now.tv_usec);
		BHLOG(LOGLORAR) printf("IRQ%i<%ul>: FSK-Mode - should never happen (1) (OPMode: 0x%0.2X)-> RD-OPMode Retry...\n", 
				 (unsigned char)dio, (unsigned long)tstamp, (unsigned char) readReg(RegOpMode));
		delay(200);					// wait some tome till LoRa Mode has been established
		mode = readReg(RegOpMode);	// read OpMode again;
	}
	
	// 0. final LoRa mode check for ISR handling
	if( (mode & OPMODE_LORA) != 0) {		// Really LORA modem Mode ?
		flags = readReg(LORARegIrqFlags);

		tstamp = (unsigned long)(now.tv_sec*1000000 + now.tv_usec); // get TimeStamp in seconds
		BHLOG(LOGLORAW) printf("  IRQ[%i]-%i<%ul>: LoRa-IRQ flags: 0x%02X - Mask:0x%02X: ", 
			(int)this->mset.modemid, (unsigned char)dio, (unsigned long)tstamp, (unsigned char)flags, (unsigned char)readReg(LORARegIrqFlagsMask));

		// This flags are not really of interest for us here
		//		if((flags & IRQ_LORA_HEADER_MASK) == IRQ_LORA_HEADER_MASK) printf(" ValidHeader");
		//		if((flags & IRQ_LORA_FHSSCH_MASK) == IRQ_LORA_FHSSCH_MASK) printf(" FHSSChannel");
	
		gwt.cp_nb_rx_rcv++;		// statistics for BIoTApp(): incr. # received pkgs.
		
		// 1. IRQ in TX Mode: TX DONE assumed
		if( (mset.currentMode & OPMODE_TX)== OPMODE_TX){	
			//if(  flags & IRQ_LORA_TXDONE_MASK){
				// TXDone expected -> save exact tx time
				mset.txend = tstamp - mset.txstart - LORA_TXDONE_FIXUP; // TXDONE FIXUP
				BHLOG(LOGLORAR) printf(" TXDONE <%u ticks = %.4fsec.>", (unsigned long)mset.txend, (float) (mset.txend / OSTICKS_PER_SEC));
				fflush(stdout);

				TXDoneFlag =1;		// tell the user land : TX Done
				writeReg(LORARegIrqFlags, IRQ_LORA_TXDONE_MASK);		// clear TXDone IRQ flag
				BHLOG(LOGLORAW) printf("\n");
				--gwt.cp_nb_rx_rcv;		// TXDone Pkgs not counted as Received pkg.-> only RX
			// }
		// 3. IRQ in RX Mode: RX DONE -> Check for new received valid packet
		} else if((flags & IRQ_LORA_RXDONE_MASK) || flags==0) {  // receiving a LoRa package ?
			// Save exact RXDone time (needed for start of RX1 window)
			mset.rxtime = tstamp;
			if(ccfg->bw == BW125) 
				mset.rxtime -= LORA_RXDONE_FIXUP[ccfg->sf];	// correct used time by ISR routine
			BHLOG(LOGLORAR) printf(" RXDone v 0x00");

			writeReg(LORARegIrqFlags, IRQ_LORA_RXDONE_MASK); // Quit RXDone IRQ flag

			// read the Radio-payload FIFO -> get package length
			// LoRa (max) payload length' (in implicite header mode) or 
			// default: 'Number of payload bytes of latest packet received' 
			mset.rxdlen = (readReg(LORARegModemConfig1) & SX1272_MC1_IMPLICIT_HEADER_MODE_ON) ?
					 readReg(LORARegPayloadLength) : readReg(LORARegRxNbBytes);
			BHLOG(LOGLORAR) printf(" (0x%0.2X Byte) ", (unsigned char)mset.rxdlen);
			
			if((mset.currentMode & OPMODE_RX) == OPMODE_RX){ // in RX_Cont Mode ? 
				// Workaround: In a RXCont session an RXDone has another meaning
				// Have to wait for Payload RX complete status
				// ToDO: Root cause analysis
				BHLOG(LOGLORAR) printf("RXContWait(%dms) ", (unsigned char)mset.rxdlen);
				delay(mset.rxdlen+20);	// wait for each received FIFO-Byte (assumed Ts=1ms) + some buffer 
			}

			// 4. CRC Check
			flags = readReg(LORARegIrqFlags);	// ReRead flags for Payload-CRC error check
			if((flags & IRQ_LORA_CRCERR_MASK) == IRQ_LORA_CRCERR_MASK){
				writeReg(LORARegIrqFlags, IRQ_LORA_CRCERR_MASK); // clear CRCErr IRQ flag
				BHLOG(LOGLORAW) printf(" CRCError -> ignore IRQ\n");
				// ToDO How to tell user about this case ?
				gwt.cp_nb_rx_crc++;
//				setopmode(OPMODE_STANDBY); // Force Idle Mode -> results in recfg in main loop to RXCont

				
			}else{ 
			// 5a. Get RSSI & SNR Status
				BHLOG(LOGLORAR) printf("\n");
	            // Interprete rx quality parameters SNR & RSSI
				// dB Calculation is based on SemTech SX127x specification page 87

				int PSNR;	
				byte value = readReg(LORARegPktSnrValue);
				if( value & 0x80 ){ // The SNR sign bit is 1
					value = ( ( ~value + 1 ) & 0xFF ) >> 2; // Invert and divide by 4
					PSNR = -value;
				} else {
					PSNR = ( value & 0xFF ) >> 2;           // just divide by 4
				}

				int PRSSI;	// PacketStrength in [dBm]
				int16_t rssi = readReg(LORARegPktRssiValue); // get averaged PacketRSSI value
				// need dBm adjustment for RSSI values at HF or LF Channel to have better linearity
                if( PSNR < 0 ){
					if( ccfg->freq > RF_MID_BAND_THRESH ){	// HF Mode
						PRSSI = RSSI_OFFSET_HF + rssi + ( rssi >> 4 ) + PSNR;
					}else{								// LF Mode
						PRSSI = RSSI_OFFSET_LF + rssi + ( rssi >> 4 ) + PSNR;
					}
				}else{ // PSNR >=0
					if( ccfg->freq > RF_MID_BAND_THRESH ){	// HF Mode
						PRSSI = RSSI_OFFSET_HF + rssi + ( rssi >> 4 );
					}else{								// LF Mode
						PRSSI = RSSI_OFFSET_LF + rssi + ( rssi >> 4 );
					}
				}
				BHLOG(LOGLORAW) printf(" Pkt-RSSI:%i, RSSI:%i, SNR: %i, Len: %iBy., OPMode(Reg:0x%02X) %s\n",  
					(int) rssi, (int)PRSSI, (int)PSNR, (int)mset.rxdlen, readReg(RegOpMode), rxloraOMstring[mset.currentMode & OPMODE_MASK]);
				mset.rssi = PRSSI;
				mset.snr  = PSNR;

				// 5b. SNR Check: SNR above threshold assures valid xfer packets of own clients in range -> ours
				//     other values might be packets with different spreading factors -> not ours
				if(PSNR >= RF_SNR_THRESH){	// can we assume a valid payload in the queue ?
					// - set FIFO read address pointer to pkg begin
					writeReg(LORARegFifoAddrPtr, readReg(LORARegFifoRxCurrentAddr)); 

					
					// 6. Check Package size: in range of BeeIoT WAN protocol specification ?
					if (mset.rxdlen < BIoT_HDRLEN || mset.rxdlen > MAX_PAYLOAD_LENGTH) {
						// Non BeeIoT Package -> store for future analysis / test purpose
						readBuf(RegFifo, (byte *) mset.rxbuffer, mset.rxdlen);				
						BHLOG(LOGLORAR) printf("  IRQ%d: New RXPkg size out of BIoTWAN range: %iBy -> ignored\n", 
								(unsigned char)dio, (int) mset.rxdlen);
						//	hexdump((byte *) & rxbuffer, (byte) mset.rxdlen);

						// ToDO: further processing of this proprietary message ?
						// by now ignored...
						gwt.cp_nb_rx_bad++;	// bad pkg
						
					// 7. Finally: fetch the received payload from FiFo to given RX Queue buffer
					}else{ // seems to be a valid BeeIoT package by length, move pkg from SX-Queue to RXQueue

						BHLOG(LOGLORAR) printf("  IRQ%d: Get BeeIoT RXDataPkg - len=%iBy\n",
								(unsigned char)dio, (int)mset.rxdlen);
						
						// Create MsgBuffer & Copy Payload directly from RegFiFo to MsgBuffer->pkg (incl. MIC)
						MsgBuffer mb(this->mset.modemid, mset.rssi, mset.snr);
						mb.setpkgfifo(this, (byte)RegFifo, (int)mset.rxdlen);
						
						BHLOG(LOGQUE) printf("\n  IRQ%d: PkgHD: 0x", (unsigned char)dio);
						BHLOG(LOGQUE) mb.printpkg(BIoT_HDRLEN);		// Show Header field of package (only)
						BHLOG(LOGQUE) printf("\n");

						// Signal to Userland: New Entry in RX Queue: Size++
						mset.gwq->PushMsg(mb);	// Move (!) MsgBuffer to Queue-end + implicite MB delete

					} // Got BeeIoT Pkg
				} // PSNR threshold check
			}// !CRC check

			
		// 8. RXTOUT-Check
        } else if((flags & IRQ_LORA_RXTOUT_MASK) == IRQ_LORA_RXTOUT_MASK){
            // indicate timeout
            mset.rxdlen = 0;
			writeReg(LORARegIrqFlags, IRQ_LORA_RXTOUT_MASK);		// clear RXTOUT IRQ flag
			mset.rxtime = tstamp;
			BHLOG(LOGLORAW) printf(" RXTout\n");
			// ToDO How to tell user about this case ?
			gwt.cp_nb_rx_bad++;	// bad pkg
		} 
		// end of IRQ Flag validation chain

        // 9. Clear/ack all radio IRQ flags and close masking
        writeReg(LORARegIrqFlagsMask, 0xFF);   // mask all radio IRQs
        writeReg(LORARegIrqFlags, 0xFF);        // clear radio IRQ flags

		
	// 10. FSK modem IRQ -> should never happen
    } else { 
		// Based on Semtech Errata: Observed in case of SPI line instability 
		// -> full reset of modem helps
        struct timeval now;
        gettimeofday(&now, 0);
        tstamp = (uint32_t)(now.tv_sec*1000000 + now.tv_usec);

		BHLOG(LOGLORAW) printf("IRQ<%ul>: FSK-IRQ%d - should never happen (2)(OPMode: 0x%0.2X) -> skipped\n", 
					(unsigned long)tstamp, (unsigned char)dio, (unsigned char) readReg(RegOpMode));

		// clear/ack all radio IRQ flags and close masking
		//        writeReg(LORARegIrqFlagsMask, 0xFF);   // mask all radio IRQs
		//        writeReg(LORARegIrqFlags, 0xFF);        // clear radio IRQ flags

		// Force LoRa Mode for this ISR session just to be sure
		BHLOG(LOGLORAR) printf("IRQ%d: Force ModemType from FSK to last known LoRa-Mode 0x%02X!\n",
				(unsigned char)dio, (unsigned char)mset.currentMode);

		setLoraMode();	// Force OPMode to Lora + HF On + no FKS reg access + OM:SLEEP
		// SetupLoRa(); // may be this is also needed ? Full Reset + Reconfig
    } // FSK IRQ
	
	setopmode(OPMODE_SLEEP); // Force SLEEP Mode
	fflush(stdout);
} // end of IRQ ISR



void Radio::MyIRQ0(void) {
  BHLOG(LOGLORAR) printf("IRQ[%i] at DIO 0 (level %i)\n", (int) mset.modemid,(unsigned char) mset.irqlevel);
  if (mset.irqlevel==0) {
	hal_disableIRQs();
	myradio_irq_handler(0);			// instead receivepacket();
	hal_enableIRQs();
	return;
  }
}

void Radio::MyIRQ1(void) {
  BHLOG(LOGLORAR) printf("IRQ[%i] at DIO 1 (level %i)\n", (int) mset.modemid,(unsigned char) mset.irqlevel);
  if (mset.irqlevel==0){
	hal_disableIRQs();
    myradio_irq_handler(1);
	hal_enableIRQs();
  }
}

void Radio::MyIRQ2(void) {
  BHLOG(LOGLORAR) printf("IRQ[%i] at DIO 2 (level %i)\n", (int) mset.modemid,(unsigned char) mset.irqlevel);
  if (mset.irqlevel==0){
	hal_disableIRQs();
    myradio_irq_handler(2);
	hal_enableIRQs();
  }
}


void Radio::hal_disableIRQs (void) {
	if(++mset.irqlevel > 0){
//    printf("    Disabled IRQs(%d)\n", mset.irqlevel);
	}
}

void Radio::hal_enableIRQs (void) {
    if(--mset.irqlevel == 0){
//      printf("    Enabled IRQs !\n");
	}
}


void Radio::Radio_AttachIRQ(uint8_t irq_pin, int irqtype, void (*ISR_callback)(void)){
    wiringPiISR(irq_pin, irqtype, ISR_callback);	// C-function call
}

void Radio::init(){
	
//	Radio_AttachIRQ(modemp->iopins.sxdio0, INT_EDGE_RISING, []{this->MyIRQ0(); } );
//	Radio_AttachIRQ(modemp->iopins.sxdio0, INT_EDGE_RISING, reinterpret_cast<void(*)()>(MyIRQ0()) );
	
}

// Assign interrupt handler to IRQ GPIO port
// Using global gwset[x].modem allows final IRQ handler to run completely in private Modem object context (!)
void isr_init(modemcfg_t * mod) {
	iopins_t *pins	= & mod->iopins;	// get GPIO Pin Structure of selected modem instance 
	byte mid		= mod->modemid;		// get current modem id (expected to be limited to MAXGW by caller !
	Radio *lora		= gwtab.modem[mid];	// get Instance ptr.

	// We need a ptr type: void (*ISR_callback)(void) from a member function to use C-Function: wiringPiISR();
	// But Following Lambda function accepts only static modem expressions (like gwset[x].modem).
	// Calculated references at runtime like 'mod->modem' or 'gwset[mid].modem' or combined with this
	// results in compilation errors (=> {expression} not captured)

	switch (mid){
	case 0:
		lora->Radio_AttachIRQ(pins->sxdio0, INT_EDGE_RISING, []{gwtab.modem[0]->MyIRQ0();} );
		lora->Radio_AttachIRQ(pins->sxdio1, INT_EDGE_RISING, []{gwtab.modem[0]->MyIRQ1();} );
		lora->Radio_AttachIRQ(pins->sxdio2, INT_EDGE_RISING, []{gwtab.modem[0]->MyIRQ2();} );
		BHLOG(LOGLORAW) printf("  ISR_Init(%i): --- ISR on DIO0+1+2 assigned ---\n", (int) mid);
		break;
	case 1:
		lora->Radio_AttachIRQ(pins->sxdio0, INT_EDGE_RISING, []{gwtab.modem[1]->MyIRQ0();} );
		lora->Radio_AttachIRQ(pins->sxdio1, INT_EDGE_RISING, []{gwtab.modem[1]->MyIRQ1();} );
		lora->Radio_AttachIRQ(pins->sxdio2, INT_EDGE_RISING, []{gwtab.modem[1]->MyIRQ2();} );
		BHLOG(LOGLORAW) printf("  ISR_Init(%i): --- ISR on DIO0+1+2 assigned ---\n", (int) mid);
		break;
	// Unfortunately this case list must be expanded manually to keep  static expressions.
	// only MAXGW entries supported
	default:
		BHLOG(LOGLORAW) printf("  ISR_Init: Unsupported Lora modem idx %i => No ISRs assigned\n", (int) mid);
		return;
	}
}


//*******************************************************************************
// link GW Queue to modem session (one per modem)
void Radio::assign_gwqueue(MsgQueue * gwq){
	mset.gwq = gwq;	// store reference to GW Message Queue
};	
	