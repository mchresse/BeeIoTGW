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
 * This file provides incorporated routines out of the great LMIC project 
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

#include <string.h>
#include <stdio.h>
#include <stddef.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <errno.h>
#include <cstdlib>
#include <wiringPi.h>
#include <wiringPiSPI.h>

#include "BeeIoTWan.h"
#include "beelora.h"

#include "beeiot.h"

extern unsigned int	lflags;               // BeeIoT log flag field

// ---------------------------------------- 
// Registers Mapping
#define RegFifo                                    0x00 // common
#define RegOpMode                                  0x01 // common
#define FSKRegBitrateMsb                           0x02
#define FSKRegBitrateLsb                           0x03
#define FSKRegFdevMsb                              0x04
#define FSKRegFdevLsb                              0x05
#define RegFrfMsb                                  0x06 // common
#define RegFrfMid                                  0x07 // common
#define RegFrfLsb                                  0x08 // common
#define RegPaConfig                                0x09 // common
#define RegPaRamp                                  0x0A // common
#define RegOcp                                     0x0B // common
#define RegLna                                     0x0C // common
#define FSKRegRxConfig                             0x0D
#define LORARegFifoAddrPtr                         0x0D
#define FSKRegRssiConfig                           0x0E
#define LORARegFifoTxBaseAddr                      0x0E
#define FSKRegRssiCollision                        0x0F
#define LORARegFifoRxBaseAddr                      0x0F 
#define FSKRegRssiThresh                           0x10
#define LORARegFifoRxCurrentAddr                   0x10
#define FSKRegRssiValue                            0x11
#define LORARegIrqFlagsMask                        0x11 
#define FSKRegRxBw                                 0x12
#define LORARegIrqFlags                            0x12 
#define FSKRegAfcBw                                0x13
#define LORARegRxNbBytes                           0x13 
#define FSKRegOokPeak                              0x14
#define LORARegRxHeaderCntValueMsb                 0x14 
#define FSKRegOokFix                               0x15
#define LORARegRxHeaderCntValueLsb                 0x15 
#define FSKRegOokAvg                               0x16
#define LORARegRxPacketCntValueMsb                 0x16 
#define LORARegRxpacketCntValueLsb                 0x17 
#define LORARegModemStat                           0x18 
#define LORARegPktSnrValue                         0x19 
#define FSKRegAfcFei                               0x1A
#define LORARegPktRssiValue                        0x1A 
#define FSKRegAfcMsb                               0x1B
#define LORARegRssiValue                           0x1B 
#define FSKRegAfcLsb                               0x1C
#define LORARegHopChannel                          0x1C 
#define FSKRegFeiMsb                               0x1D
#define LORARegModemConfig1                        0x1D 
#define FSKRegFeiLsb                               0x1E
#define LORARegModemConfig2                        0x1E 
#define FSKRegPreambleDetect                       0x1F
#define LORARegSymbTimeoutLsb                      0x1F 
#define FSKRegRxTimeout1                           0x20
#define LORARegPreambleMsb                         0x20 
#define FSKRegRxTimeout2                           0x21
#define LORARegPreambleLsb                         0x21 
#define FSKRegRxTimeout3                           0x22
#define LORARegPayloadLength                       0x22 
#define FSKRegRxDelay                              0x23
#define LORARegPayloadMaxLength                    0x23 
#define FSKRegOsc                                  0x24
#define LORARegHopPeriod                           0x24 
#define FSKRegPreambleMsb                          0x25
#define LORARegFifoRxByteAddr                      0x25
#define LORARegModemConfig3                        0x26
#define FSKRegPreambleLsb                          0x26
#define FSKRegSyncConfig                           0x27
#define LORARegFeiMsb                              0x28
#define FSKRegSyncValue1                           0x28
#define LORAFeiMib                                 0x29
#define FSKRegSyncValue2                           0x29
#define LORARegFeiLsb                              0x2A
#define FSKRegSyncValue3                           0x2A
#define FSKRegSyncValue4                           0x2B
#define LORARegRssiWideband                        0x2C
#define FSKRegSyncValue5                           0x2C
#define FSKRegSyncValue6                           0x2D
#define FSKRegSyncValue7                           0x2E
#define FSKRegSyncValue8                           0x2F
#define FSKRegPacketConfig1                        0x30
#define FSKRegPacketConfig2                        0x31
#define LORARegDetectOptimize                      0x31
#define FSKRegPayloadLength                        0x32
#define FSKRegNodeAdrs                             0x33
#define LORARegInvertIQ                            0x33
#define FSKRegBroadcastAdrs                        0x34
#define FSKRegFifoThresh                           0x35
#define FSKRegSeqConfig1                           0x36
#define FSKRegSeqConfig2                           0x37
#define LORARegDetectionThreshold                  0x37
#define FSKRegTimerResol                           0x38
#define FSKRegTimer1Coef                           0x39
#define LORARegSyncWord                            0x39
#define FSKRegTimer2Coef                           0x3A
#define FSKRegImageCal                             0x3B
#define FSKRegTemp                                 0x3C
#define FSKRegLowBat                               0x3D
#define FSKRegIrqFlags1                            0x3E
#define FSKRegIrqFlags2                            0x3F
#define RegDioMapping1                             0x40 // common
#define RegDioMapping2                             0x41 // common
#define RegVersion                                 0x42 // common
// #define RegAgcRef                                  0x43 // common
// #define RegAgcThresh1                              0x44 // common
// #define RegAgcThresh2                              0x45 // common
// #define RegAgcThresh3                              0x46 // common
// #define RegPllHop                                  0x4B // common
// #define RegTcxo                                    0x58 // common
#define RegPaDac                                   0x5A // common
// #define RegPll                                     0x5C // common
// #define RegPllLowPn                                0x5E // common
// #define RegFormerTemp                              0x6C // common
#define RegBitRateFrac                             0x70 // common

// ----------------------------------------
// spread factors and mode for RegModemConfig2
#define SX1272_MC2_FSK  0x00
#define SX1272_MC2_SF7  0x70
#define SX1272_MC2_SF8  0x80
#define SX1272_MC2_SF9  0x90
#define SX1272_MC2_SF10 0xA0
#define SX1272_MC2_SF11 0xB0
#define SX1272_MC2_SF12 0xC0
// bandwidth for RegModemConfig1
#define SX1272_MC1_BW_125  0x00
#define SX1272_MC1_BW_250  0x40
#define SX1272_MC1_BW_500  0x80
// coding rate for RegModemConfig1
#define SX1272_MC1_CR_4_5 0x08
#define SX1272_MC1_CR_4_6 0x10
#define SX1272_MC1_CR_4_7 0x18
#define SX1272_MC1_CR_4_8 0x20
#define SX1272_MC1_IMPLICIT_HEADER_MODE_ON 0x04 // required for receive
#define SX1272_MC1_RX_PAYLOAD_CRCON        0x02
#define SX1272_MC1_LOW_DATA_RATE_OPTIMIZE  0x01 // mandated for SF11 and SF12
// transmit power configuration for RegPaConfig
#define SX1272_PAC_PA_SELECT_PA_BOOST 0x80
#define SX1272_PAC_PA_SELECT_RFIO_PIN 0x00


// sx1276 RegModemConfig1
#define SX1276_MC1_BW_125                0x70
#define SX1276_MC1_BW_250                0x80
#define SX1276_MC1_BW_500                0x90
#define SX1276_MC1_CR_4_5            0x02
#define SX1276_MC1_CR_4_6            0x04
#define SX1276_MC1_CR_4_7            0x06
#define SX1276_MC1_CR_4_8            0x08

#define SX1276_MC1_IMPLICIT_HEADER_MODE_ON    0x01 
                                                    
// sx1276 RegModemConfig2          
#define SX1276_MC2_RX_PAYLOAD_CRCON        0x04

// sx1276 RegModemConfig3          
#define SX1276_MC3_LOW_DATA_RATE_OPTIMIZE  0x08
#define SX1276_MC3_AGCAUTO                 0x04

// preamble for lora networks (nibbles swapped)
#define LORA_MAC_PREAMBLE                  0x34

#define RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG1 0x0A
#define RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG2 0x70	// fo SX1276 only


// ---------------------------------------- 
// Constants for radio registers
#define OPMODE_LORA      0x80
#define OPMODE_MASK      0x07
#define OPMODE_LFMASK	 0x08

#define OPMODE_SLEEP     0x00
#define OPMODE_STANDBY   0x01
#define OPMODE_FSTX      0x02
#define OPMODE_TX        0x03
#define OPMODE_FSRX      0x04
#define OPMODE_RX        0x05
#define OPMODE_RX_SINGLE 0x06 
#define OPMODE_CAD       0x07 

// ----------------------------------------
// Bits masking the corresponding IRQs from the radio
#define IRQ_LORA_RXTOUT_MASK 0x80
#define IRQ_LORA_RXDONE_MASK 0x40
#define IRQ_LORA_CRCERR_MASK 0x20
#define IRQ_LORA_HEADER_MASK 0x10
#define IRQ_LORA_TXDONE_MASK 0x08
#define IRQ_LORA_CDDONE_MASK 0x04
#define IRQ_LORA_FHSSCH_MASK 0x02
#define IRQ_LORA_CDDETD_MASK 0x01

#define IRQ_FSK1_MODEREADY_MASK         0x80
#define IRQ_FSK1_RXREADY_MASK           0x40
#define IRQ_FSK1_TXREADY_MASK           0x20
#define IRQ_FSK1_PLLLOCK_MASK           0x10
#define IRQ_FSK1_RSSI_MASK              0x08
#define IRQ_FSK1_TIMEOUT_MASK           0x04
#define IRQ_FSK1_PREAMBLEDETECT_MASK    0x02
#define IRQ_FSK1_SYNCADDRESSMATCH_MASK  0x01
#define IRQ_FSK2_FIFOFULL_MASK          0x80
#define IRQ_FSK2_FIFOEMPTY_MASK         0x40
#define IRQ_FSK2_FIFOLEVEL_MASK         0x20
#define IRQ_FSK2_FIFOOVERRUN_MASK       0x10
#define IRQ_FSK2_PACKETSENT_MASK        0x08
#define IRQ_FSK2_PAYLOADREADY_MASK      0x04
#define IRQ_FSK2_CRCOK_MASK             0x02
#define IRQ_FSK2_LOWBAT_MASK            0x01

// ----------------------------------------
// DIO function mappings                D0D1D2D3
#define MAP_DIO0_LORA_RXDONE   0x00  // 00------
#define MAP_DIO0_LORA_TXDONE   0x40  // 01------
#define MAP_DIO1_LORA_RXTOUT   0x00  // --00----
#define MAP_DIO1_LORA_NOP      0x30  // --11----
#define MAP_DIO2_LORA_NOP      0x0C  // ----11--

#define MAP_DIO0_FSK_READY     0x00  // 00------ (packet sent / payload ready)
#define MAP_DIO1_FSK_NOP       0x30  // --11----
#define MAP_DIO2_FSK_TXNOP     0x04  // ----01--
#define MAP_DIO2_FSK_TIMEOUT   0x08  // ----10--


// FSK IMAGECAL defines
#define RF_IMAGECAL_AUTOIMAGECAL_MASK               0x7F
#define RF_IMAGECAL_AUTOIMAGECAL_ON                 0x80
#define RF_IMAGECAL_AUTOIMAGECAL_OFF                0x00  // Default

#define RF_IMAGECAL_IMAGECAL_MASK                   0xBF
#define RF_IMAGECAL_IMAGECAL_START                  0x40

#define RF_IMAGECAL_IMAGECAL_RUNNING                0x20
#define RF_IMAGECAL_IMAGECAL_DONE                   0x00  // Default


// RADIO STATE
/*!
 * Constant values need to compute the RSSI value
 */
#define RSSI_OFFSET_LF              -164.0
#define RSSI_OFFSET_HF              -157.0
#define RF_MID_BAND_THRESH          525000000

// (initialized by radio_init(), used by radio_rand1())
static u1_t randbuf[16];


#define LNA_RX_GAIN (0x20|0x00)		// MaxLNAGain + NoBoost
#define LNA_MAX_GAIN (0x20|0x03)		// MaxLNAGain + Boost(150%) -> Default for SX1276

static const u1_t rxlorairqmask[] = {
    [RXMODE_SINGLE] = IRQ_LORA_RXDONE_MASK|IRQ_LORA_RXTOUT_MASK,
    [RXMODE_SCAN]   = IRQ_LORA_RXDONE_MASK,
    [RXMODE_RSSI]   = 0x00,
};
// in case of FreqHopOn -> add IRQ_LORA_FHSSCH_MASK here and check in ISR2

const char * rxloraOMstring[] = {
	[OPMODE_SLEEP]	= "SLEEP",
	[OPMODE_STANDBY]= "STANDBY",
	[OPMODE_FSTX]	= "FSTX",
	[OPMODE_TX]		= "TX",
	[OPMODE_FSRX]	= "FSRX",
	[OPMODE_RX]		= "RXCONT",
	[OPMODE_RX_SINGLE]="RXSINGLE",
	[OPMODE_CAD]	= "CAD",
};

//*****************************************************************************
// RADIO layer cfg settings used by MAIN()
//*****************************************************************************
long freq	= FREQ;					// =EU868_F1..9,DN (EU868_F1: 868.1MHz)
s1_t pw		= TXPOWER;				// =2-16  TX PA Mode (14)
sf_t sf		= SPREADING;			// =0..8 Spreading factor FSK,7..12,SFrFu (1:SF7)
bw_t bw		= SIGNALBW;				// =0..3 RFU Bandwidth 125-500 (0:125kHz)
cr_t cr		= CODING;				// =0..3 Coding mode 4/5..4/8 (0:4/5)
byte ih		= IHDMODE;				// =1 implicite Header Mode (0)
u1_t ihlen	= IHEADERLEN;			// =0..n if IH -> header length (0)
u1_t nocrc	= NOCRC;				// =0/1 no CRC check used for Pkg (0)
u1_t noRXIQinversion = NORXIQINV;	// =0/1 flag to switch RX+TX IQinv. on/off (1)

byte currentMode = OPMODE_LORA | OPMODE_SLEEP; // current LoRa Modem Op.Mode
									// (start LoRa Modem in SLEEP Mode)

bool sx1276;					// =0(SX1276), =1(SX1272) (0)
extern byte BeeIotRXFlag;			// =0..MAXRXPKG RX Queue length (# of unparsed RX pkgs.)
extern byte BeeIotTXFlag;			// =1 TXDone received by ISR
extern byte RXPkgIsrIdx;            // index on next RX Queue Package for ISR callback Write
extern beeiotpkg_t MyRXData[];		// RX Queue Buffer

int irqlevel = 0;					// =0..n IRQ Enable semaphor: (0: IRQs allowed) 

// used in upload_function()
extern uint32_t cp_nb_rx_rcv;			// # received packages -> Statistics for BIoTApp()


// used by ISR routine
byte snr;
byte rssi;
byte rxframe[256];
byte rxdlen = 0;
byte txframe[256];
byte txdlen = 0;

struct timeval now;		// current tstamp used each time a time check is done
unsigned long txstart;		// tstamp when TX Mode was entered
unsigned long txend;			// Delta: now - txstart
unsigned long rxtime;		// tstamp when last rx package arrived

//*****************************************************************************
// local RADIO layer function prototypes

static void rxlora (u1_t rxmode, int rxto);
static void radio_init ();




//*****************************************************************************
// RADIO layer function
//*****************************************************************************
static void hal_pin_nss (u1_t val) {
    digitalWrite(LORAcs, val);
}

// perform 8-bit SPI transaction with radio
// write given byte outval to radio, read byte from radio and return value.
static u1_t hal_spi (u1_t out) {
    u1_t res = wiringPiSPIDataRW(0, &out, 1);
	if (res <0){
		fprintf(stderr, "HAL: Cannot send data on SPI: %s\n", strerror(errno));
		// exit ?)
	}
    return out;
}

// SPI transfer with address and one single byte.
u1_t hal_spi_single (u1_t address, u1_t out) {
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

static void writeReg (u1_t addr, u1_t data ) {
    hal_pin_nss(0);
    hal_spi(addr | 0x80);
    hal_spi(data);
    hal_pin_nss(1);
}

static u1_t readReg (u1_t addr) {
    hal_pin_nss(0);
    hal_spi(addr & 0x7F);
    u1_t val = hal_spi(0x00);
    hal_pin_nss(1);
    return val;
}

static void writeBuf (u1_t addr, xref2u1_t buf, u1_t len) {
    hal_pin_nss(0);
    hal_spi(addr | 0x80);
    for (u1_t i=0; i<len; i++) {
        hal_spi(buf[i]);
    }
    hal_pin_nss(1);
}

static void readBuf (u1_t addr, xref2u1_t buf, u1_t len) {
    hal_pin_nss(0);
    hal_spi(addr & 0x7F);
    for (u1_t i=0; i<len; i++) {
        buf[i] = hal_spi(0x00);
    }
    hal_pin_nss(1);
}

void opmode (u1_t mode) {
	mode &= OPMODE_MASK|OPMODE_LORA;
	currentMode = (readReg(RegOpMode) & ~OPMODE_MASK) | mode;
    writeReg(RegOpMode, currentMode);	// inkl. last LF mode status
	BHLOG(LOGLORAR) printf("    Radio: New OpMode: %s (0x%02X)\n", 
			rxloraOMstring[mode&OPMODE_MASK], (unsigned int)currentMode);
	currentMode &= (OPMODE_LORA | OPMODE_MASK);		// Filter LF Mode
}

void opmodeLora() {
    // def:LoRa with LF Mode - for SX1276 only !
	currentMode = OPMODE_LORA | OPMODE_SLEEP; // Start in SLeep Mode +
	writeReg(RegOpMode, currentMode);		  // HF Mode, no shared FSK Reg Access
    BHLOG(LOGLORAR) printf("    Radio: LoRa-Modem preset\n");
}


// configure LoRa modem (cfg1, cfg2)
static void configLoraModem () {
    u1_t mc1 = 0, mc2 = 0, mc3 = 0;
	int sbw; 
	const char *scr;

    // set ModemConfig1
        switch (bw) {
        case BW125: mc1 |= SX1276_MC1_BW_125; sbw=125; break;
        case BW250: mc1 |= SX1276_MC1_BW_250; sbw=250; break;
        case BW500: mc1 |= SX1276_MC1_BW_500; sbw=500; break;
        default:		//  ASSERT(0);
			BHLOG(LOGLORAR) printf("    Radio: configLoraModem:  Warning_wrong BW setting: %i kHz\n", sbw);
        }
        switch( cr ) {
        case CR_4_5: mc1 |= SX1276_MC1_CR_4_5; scr = "4/5"; break;
        case CR_4_6: mc1 |= SX1276_MC1_CR_4_6; scr = "4/6"; break;
        case CR_4_7: mc1 |= SX1276_MC1_CR_4_7; scr = "4/7"; break;
        case CR_4_8: mc1 |= SX1276_MC1_CR_4_8; scr = "4/8"; break;
		default:		//  ASSERT(0);
			BHLOG(LOGLORAR) printf("    Radio: configLoraModem:  Warning_unknown CR value: %i\n",(int) scr);
        }

        if (ih) {
            mc1 |= SX1276_MC1_IMPLICIT_HEADER_MODE_ON;
            writeReg(LORARegPayloadLength, ihlen); // required length
        }
        writeReg(LORARegModemConfig1, mc1);

    // set ModemConfig2
        mc2 = (SX1272_MC2_SF7 + ((sf-1)<<4));
        if (!nocrc) { 
            mc2 |= SX1276_MC2_RX_PAYLOAD_CRCON;
        }
        writeReg(LORARegModemConfig2, mc2);
        
    // set ModemConfig3
        mc3 = SX1276_MC3_AGCAUTO;
        if( ((bw == BW125) && (sf == SF11 || sf == SF12)) ||
			 (bw == BW250) && (sf == SF12)){
            mc3 |= SX1276_MC3_LOW_DATA_RATE_OPTIMIZE;
        }
        writeReg(LORARegModemConfig3, mc3);
		
		BHLOG(LOGLORAR) printf("    ConfigLoRa: mc1=0x%02X, mc2=0x%02X, mc3=0x%02X, Channel= %.6lf MHz\n",
				(unsigned char)mc1, (unsigned char)mc2, (unsigned char)mc3, (double)freq/1000000);
}

static void configChannel () {
    // set frequency: FQ = (FRF * 32 Mhz) / (2 ^ 19)
    u8_t frf = ((u8_t)freq << 19) / 32000000;
    writeReg(RegFrfMsb, (u1_t)(frf>>16));
    writeReg(RegFrfMid, (u1_t)(frf>> 8));
    writeReg(RegFrfLsb, (u1_t)(frf>> 0));
//	printf("    ConfigChannel= %.6lf MHz\n", (double)freq/1000000);
}

static void configPower () {
	// no boost used for now
    if(pw >= 17) {
        pw = 15;
    } else if(pw < 2) {
        pw = 2;
    }
    // ToDo: check board type for BOOST pin

	writeReg(RegPaRamp, (readReg(RegPaRamp) & 0xF0) | 0x08); // set PA ramp-up time 50 uSec
    writeReg(RegPaConfig, (u1_t)(0x80|(pw&0xf)));
    writeReg(RegPaDac, readReg(RegPaDac)|0x4);
	BHLOG(LOGLORAR) printf("    ConfigPower: PA_RampUp=50usec., PA_CONFIG=0x%02X, PA_DAC=0x%02X\n", 
		(byte)readReg(RegPaConfig), (byte)readReg(RegPaDac));
}


// get random seed from wideband noise rssi
void radio_init () {
	// initial init: no IRQ request allowed
	irqlevel = 0;
    hal_disableIRQs();

     // seed 15-byte randomness via noise rssi
    rxlora(RXMODE_RSSI,0);
	BHLOG(LOGLORAR) printf("  radio_init(): get random field values:\n");
    while( (readReg(RegOpMode) & OPMODE_MASK) != OPMODE_RX ); // continuous rx ?
    for(int i=1; i<16; i++) {
		BHLOG(LOGLORAR) printf("  %2d:", i);
        for(int j=0; j<8; j++) {
            u1_t b; // wait for two non-identical subsequent least-significant bits
            while( (b = readReg(LORARegRssiWideband) & 0x01) == (readReg(LORARegRssiWideband) & 0x01) );
            randbuf[i] = (randbuf[i] << 1) | b;
			BHLOG(LOGLORAR) printf(" %0.2X", (unsigned char)randbuf[i]);
        }
		BHLOG(LOGLORAR) printf("\n");
    }
    randbuf[0] = 16; // set initial index
  
    opmode(OPMODE_SLEEP);
	writeReg(LORARegIrqFlagsMask, 0xFF);

	// recover cfg. from RSSI read
	configLoraModem();			// Preset Reg_Config 1-3

    hal_enableIRQs();
	BHLOG(LOGLORAW) printf("  radio_init(): SX dev. detected and reset, randbuf[] initialized\n");
}

// return next random byte derived from seed buffer
// (buf[0] holds index of next byte to be returned)
static u1_t radio_rand1 () {
    u1_t i = randbuf[0];
    if( i==16 ) {
        os_aes(AES_ENC, (xref2u1_t)randbuf, 16); // encrypt seed with any key
        i = 0;
    }
    u1_t v = randbuf[i++];
    randbuf[0] = i;
    return v;
}

static u1_t radio_rssi () {
    hal_disableIRQs();
    u1_t r = readReg(LORARegRssiValue);
    hal_enableIRQs();
    return r;
}

byte LoRa_random(){
  return readReg(LORARegRssiWideband);
}


//******************************************************************************
// Init LoRa Modem by BeeIoTWAN Defaults: (on SX1276 chip only)
// - Reset Modem chip and read version type (SX1276 = 0x12)
// - Set OpMode SLEEP -> needed to change any cfg.
// - Set frequency REG_FRF_MSB.._LSB (0x06-0x08) <= 'FREQ' (typ.: EU868_F1)
// - Set LoRa SyncWord -> for us: private 0x12, (LoRaWan 0x34)
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
//******************************************************************************
void SetupLoRa(){
	byte mc2;

	// Reset all Channel config parameters to BeeIoT-WAN defaults
	freq	= FREQ;					// =EU868_F1..9,DN (EU868_F1: 868.1MHz)
	pw		= TXPOWER;				// =2-16  TX PA Mode (14)
	sf		= SPREADING;			// =0..8 Spreading factor FSK,7..12,SFrFu (1:SF7)
	bw		= SIGNALBW;				// =0..3 RFU Bandwidth 125-500 (0:125kHz)
	cr		= CODING;				// =0..3 Coding mode 4/5..4/8 (0:4/5)
	ih		= IHDMODE;				// =1 implicite Header Mode (0)
	ihlen	= IHEADERLEN;			// =0..n if IH -> header length (0)
	nocrc	= NOCRC;				// =0/1 no CRC check used for Pkg (0)
	noRXIQinversion = NORXIQINV;	// =0/1 flag to switch RX+TX IQinv. on/off (1)

	BHLOG(LOGLORAR) printf("  SetupLora(): Reset SX device\n");
	// reset RFM96 module
    digitalWrite(LORArst, HIGH);
    delay(100);
    digitalWrite(LORArst, LOW);
    delay(100);
    digitalWrite(LORArst, HIGH);
    delay(100);

// check SX127x chip version
    u1_t version = readReg(RegVersion);
    if (version == 0x22) {	// sx1276 ?
        BHLOG(LOGLORAR) printf("  SetupLora(): SX1272 detected\n");
        sx1276 = false;
		std::exit(-1);	// not supported
    } else {        // sx1276 ?
		if (version == 0x12) { // yes: sx1276
            BHLOG(LOGLORAW) printf("  SetupLora(): SX1276 detected\n");
            sx1276 = true;
        } else {
            BHLOG(LOGLORAR) printf("  SetupLora():Unrecognized transceiver:");
            BHLOG(LOGLORAR) printf("Version: 0x%x\n", (unsigned char) version);
            std::exit(-1);  // not supported
        }
    }

	// Set RFM96 Module to sleep mode for configuration
	opmodeLora();				// already results in SLEEP Mode
    opmode(OPMODE_SLEEP);		// now we have 'currentMode' also up to date

	configChannel();			// set Channel Frequency
	configLoraModem();			// Preset Reg_Config 1-3

    writeReg(LORARegSyncWord, LoRaWANSW); // LoRaWAN public sync word


    if (sf == SF10 || sf == SF11 || sf == SF12) {
        writeReg(LORARegSymbTimeoutLsb,RX_TOUT_LSB);	// RX_TOut_LSB = 5 x Tsymbol
    } else {
		// give 8 symbols time for preamble detection
        writeReg(LORARegSymbTimeoutLsb,0x08);	// RX_TOut_LSB = 8 x Tsymbol
    }
	writeReg(LORARegPreambleMsb, 0x00);
	writeReg(LORARegPreambleLsb, 0x08);
	

	// Sets the maximum possible downlink payload size to 64 bytes. Packets with
	// payload greater than this threshold will not be demodulated. 
	// Receiver will immediately go back to “stand-by” low power mode
	writeReg(LORARegPayloadMaxLength, MAX_PAYLOAD_LENGTH);	// def. 64 Byte TX data
    writeReg(LORARegPayloadLength, 0x40);			// typ. 64 Byte

//    writeReg(LORARegHopPeriod,0xFF);	// set Freq.Hopping period to max. time -> 255 symbols
    writeReg(LORARegHopPeriod,0x00);		// No implicite Frequ. Hopping supported
	// RegPllHop(0x44) remains unchanged (default)

	// Reset FiFo RD/WR Ptr to start of RX field base
    writeReg(LORARegFifoAddrPtr, readReg(LORARegFifoRxBaseAddr));
    writeReg(RegLna, LNA_RX_GAIN);  // Max LNA gain: G1 + DefLNACurr + NoBoost
	writeReg(RegPaRamp, (readReg(RegPaRamp) & 0xF0) | 0x08); // set PA ramp-up time 50 uSec

	// on Cont. RX Mode: DIO0 Active if SyncAddress bits are valid
	// IRQ routine have to wait for Payload ready before reading Payload data !
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

	radio_init();	// setup Radio layer houskeeping data + get Random bit field

	BHLOG(LOGLORAR) printf("  SetupLora(): Done\n");
	return;
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
void PrintLoraStatus(int logtype){
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
		printf("  IRQ Level         : %i", irqlevel);
		printf("  \n");
	}

	printf("\n");
	return;
} // end of LogLoRaStatus()



static void txlora ( byte* frame, byte dataLen) {
    // select LoRa modem (from sleep mode)
    opmodeLora();
	if((readReg(RegOpMode) & OPMODE_LORA) != OPMODE_LORA){
		BHLOG(LOGLORAW) printf("    txlora: TX in FSK Mode !\n");
		return;
	}
    // enter standby mode (required for FIFO loading))
    opmode(OPMODE_STANDBY);

    // configure LoRa modem/channel for TX (requires Standby/Sleep Mode)
    configLoraModem();	// Set Modem Cfg. registers 1,2,3
    configChannel();	// configure Channel frequency
    configPower();		// configure Antenna Output power


	// set sync word
    writeReg(LORARegSyncWord, LoRaWANSW);
    
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
    
    // now we actually start the transmission
    opmode(OPMODE_TX);
	
	gettimeofday(&now, NULL);
    txstart = (uint32_t)(now.tv_sec*1000000 + now.tv_usec); // get TimeStamp in seconds
	BHLOG(LOGLORAR) printf("    txlora(): TX pkg sent (len=0x%02X) -> TX Mode entered\n", (unsigned char) dataLen);
	BHLOG(LOGLORAR) printf("    <%ul> TXMODE, freq=%ul, len=%d, SF=%d, BW=%d, CR=4/%d, IH=%d\n",
           (unsigned long) txstart, (unsigned long) freq, (unsigned int)dataLen, (unsigned int)sf+6,
			(unsigned char) bw == BW125 ? 125 : (bw == BW250 ? 250 : 500),
			(unsigned char) cr == CR_4_5 ? 5 : (cr == CR_4_6 ? 6 : (cr == CR_4_7 ? 7 : 8)),
			(unsigned int)  ih);
}

// start transmitter (buf=LMIC.frame, len=LMIC.dataLen)
void starttx (byte* frame, byte dataLen) {
	// ASSERT( (readReg(RegOpMode) & OPMODE_MASK) == OPMODE_SLEEP );
	// basically not needed: rx routine forces STDBY Mode

	// TX for LoRa modem only
        txlora(frame, dataLen);
    // the radio will go back to STANDBY mode as soon as the TX has been finished
    // the corresponding IRQ will inform us about completion.
}

//***********************************************************
// starts LoRa receiver (time=LMIC.rxtime, timeout=LMIC.rxsyms, 
//		 				result=LMIC.frame[LMIC.dataLen])
static void rxlora (u1_t rxmode, int rxto) {
	// rxto : wait time in sec. in rx single mode
    // select LoRa modem (from sleep mode)

    opmodeLora();
	if((readReg(RegOpMode) & OPMODE_LORA) != OPMODE_LORA){
		BHLOG(LOGLORAW) printf("    rxlora: RX in FSK Mode !\n");
		return;
	}
    // enter standby mode (warm up))
    opmode(OPMODE_STANDBY);    // don't use MAC settings at startup

    if(rxmode == RXMODE_RSSI) { // use fixed settings for rssi scan
		BHLOG(LOGLORAR) printf("    rxlora: Set RXMODE_RSSI in mc1+mc2\n");
        writeReg(LORARegModemConfig1, RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG1);
        writeReg(LORARegModemConfig2, RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG2);
    } else { // Cfg. for single or continuous rx mode
	    // configure LoRa modem/channel for TX (requires Standby/Sleep Mode)
	    configLoraModem();	// Set Modem Cfg. registers 1,2,3
	    configChannel();	// configure Channel frequency
    }
    // set LNA gain
    writeReg(RegLna, LNA_RX_GAIN); 
    // set max payload size
    writeReg(LORARegPayloadMaxLength, MAX_PAYLOAD_LENGTH);
//    writeReg(LORARegPayloadLength, 0x40);		// LoRaWAN limit: 64 Byte fix
	
    // use inverted I/Q signal (prevent mote-to-mote communication)
	// new with v1.6 XXX: use flag to switch on/off inversion
    if (noRXIQinversion) {
        writeReg(LORARegInvertIQ, readReg(LORARegInvertIQ) & ~(1<<6));
    } else {
        writeReg(LORARegInvertIQ, readReg(LORARegInvertIQ)|(1<<6));
    }	

    // set symbol timeout (for single rx)
    writeReg(LORARegSymbTimeoutLsb, RX_TOUT_LSB);
    // set sync word
    writeReg(LORARegSyncWord, LoRaWANSW);
    // configure DIO mapping DIO0=RxDone DIO1=RxTout DIO2=NOP (orig)
	// on Cont. RX Mode: DIO0 Active if SyncAddress bits are valid
	// IRQ routine have to wait for Payload ready before reading Payload data !
    writeReg(RegDioMapping1, MAP_DIO0_LORA_RXDONE|MAP_DIO1_LORA_NOP|MAP_DIO2_LORA_NOP);

//	Freq Hopping not supported yet
//	writeReg(LORARegHopPeriod, 0xFF);

    // clear all radio IRQ flags
    writeReg(LORARegIrqFlags, 0xFF);
    // enable required radio IRQs
    writeReg(LORARegIrqFlagsMask, ~rxlorairqmask[rxmode]);

	// Nothing to do. There is no such pin in the Lora/GPS HAT module.
    // enable antenna switch for RX -> not needed here
	//     hal_pin_rxtx(0);

	gettimeofday(&now, NULL);
    uint32_t tstamp = (uint32_t)(now.tv_sec*1000000 + now.tv_usec); // get TimeStamp in seconds
	BHLOG(LOGLORAR) printf("    <%ul> RXMODE:%d, freq=%ul,SF=%d, BW=%d, CR=4/%d, IH=%d\n",
			(unsigned long)tstamp, (unsigned int)rxmode, (unsigned long)freq, (unsigned char)sf+6,
			(unsigned char)bw == BW125 ? 125 : (bw == BW250 ? 250 : 500),
			(unsigned char)cr == CR_4_5 ? 5 : (cr == CR_4_6 ? 6 : (cr == CR_4_7 ? 7 : 8)), 
			(unsigned char)ih);

    // now instruct the radio to receive new pkg
    if (rxmode == RXMODE_SINGLE) { // one shot with TimeOUT
//        hal_waitUntil(LMIC.rxtime); // busy wait until exact rx time
		delay(rxto*1000);			// wait in sec for incoming data
        opmode(OPMODE_RX_SINGLE);
		BHLOG(LOGLORAR) printf("    rxlora(): STart RX in SINGLE Mode for %i sec.)\n", rxto);
    } else if (rxmode == RXMODE_SCAN){ // continous rx (scan or rssi)
        opmode(OPMODE_RX); 
		BHLOG(LOGLORAR) printf("    rxlora(): Start RX in CONT Mode\n");
    } else{ // must be RXMODE_RSSI
        opmode(OPMODE_RX); 
		BHLOG(LOGLORAR) printf("    rxlora(): RSSI SCAN initiated\n");		
	}
	rxtime = 0;	// will be init by ISR at RXDONE
}

void startrx (u1_t rxmode, int rxtime) {
	// ASSERT( (readReg(RegOpMode) & OPMODE_MASK) == OPMODE_SLEEP );
	// basically not needed: rx routine forces STDBY Mode

	// LoRa modem only !
        rxlora(rxmode, rxtime);
    // the radio will go back to STANDBY mode as soon as the RX is finished
    // or timed out, and the corresponding IRQ will inform us about completion.
}

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
// My ISR funtion

// based on LMIC radio.c radio_irq_handler()
// ISR called by HAL ext IRQ handler on DIOx line
// (If != OM_LORA_RX_CONT radio goes automatically to standby mode after tx/rx operations)
void myradio_irq_handler (byte dio) {
unsigned long tstamp;

byte mode = readReg(RegOpMode);	
	
	// workaround for spurious missing LoRa OPMode flag ... just wait some ms and read again
    if( (mode & OPMODE_LORA) == 0) { // FSK Mode ?
        struct timeval now;
        gettimeofday(&now, 0);
        tstamp = (unsigned long)(now.tv_sec*1000000 + now.tv_usec);

		BHLOG(LOGLORAR) printf("IRQ<%ul>: FSK-IRQ%d - should never happen (1) (OPMode: 0x%0.2X)-> RD-OPMode Retry...\n", 
					(unsigned long)tstamp, (unsigned char)dio, (unsigned char) readReg(RegOpMode));
		delay(200);
		mode = readReg(RegOpMode);						// read again;
	}
	
	// final mode check for ISR handling
	if( (mode & OPMODE_LORA) != 0) { // LORA modem Mode ?
		byte flags = readReg(LORARegIrqFlags);

        gettimeofday(&now, 0);
        unsigned long tstamp = (unsigned long)(now.tv_sec*1000000 + now.tv_usec); // get TimeStamp in seconds
		BHLOG(LOGLORAW) printf("  IRQ%i<%ul>: LoRa-IRQ flags: 0x%02X - Mask:0x%02X: ", 
				(unsigned char)dio, (unsigned long)tstamp, (unsigned char)flags, (unsigned char)readReg(LORARegIrqFlagsMask));

// not really of interest for us
//		if((flags & IRQ_LORA_HEADER_MASK) == IRQ_LORA_HEADER_MASK) printf(" ValidHeader");
//		if((flags & IRQ_LORA_FHSSCH_MASK) == IRQ_LORA_FHSSCH_MASK) printf(" FHSSChannel");

		// TXDONE
		if(flags & IRQ_LORA_TXDONE_MASK) {
			if((currentMode & OPMODE_TX)== OPMODE_TX){ // we are in TX Mode ?
				// TXDone expected -> save exact tx time
				txend = tstamp - txstart - LORA_TXDONE_FIXUP; // TXDONE FIXUP
				BHLOG(LOGLORAW) printf(" TXDONE <%u ticks = %.4fsec.>\n", (unsigned long)txend, (float) txend / OSTICKS_PER_SEC);
				fflush(stdout);

				BeeIotTXFlag =1;		// tell the user land : TX Done
			}else{
				BHLOG(LOGLORAW) printf(" No RX Mode -> TXDONE not expected: ignored!\n"); 
			}
			writeReg(LORARegIrqFlags, IRQ_LORA_TXDONE_MASK);		// clear TXDone IRQ flag
			//			opmode(OPMODE_STANDBY); // Force Idle Mode

		// RX Queue full condition (Could be RXDONE, but no Qbuffer left)
        } else if(BeeIotRXFlag >= MAXRXPKG){ // Check RX Semaphor: RX-Queue full ?
			// same situation as: RXPkgIsrIdx+1 == RXPkgSrvIdx (but hard to check with ring buffer)
			BHLOG(LOGLORAW) printf(" RxQueue full(#%d)\n", (unsigned char) BeeIotRXFlag);
		    // Pkg has to be ignored User RX service must work harder
			rxtime = tstamp;
			// ISR flags cleared for all at the end...
//			opmode(OPMODE_STANDBY); // Force Idle Mode

		// RXDONE
		} else if((flags & IRQ_LORA_RXDONE_MASK) || flags==0) {  // receiving a LoRa package
			// Save exact rx time
			rxtime = tstamp;
            if(bw == BW125) {
              rxtime -= LORA_RXDONE_FIXUP[sf];
            }		
			BHLOG(LOGLORAW) printf(" RXDone v 0x00");

			writeReg(LORARegIrqFlags, IRQ_LORA_RXDONE_MASK); // Quit RXDone IRQ flag

            // read the payload FIFO
            rxdlen = (readReg(LORARegModemConfig1) & SX1272_MC1_IMPLICIT_HEADER_MODE_ON) ?
                readReg(LORARegPayloadLength) : readReg(LORARegRxNbBytes);
				// LoRa (max) payload length' (in implicite header mode) or 
				// default: 'Number of payload bytes of latest packetreceived' 
			BHLOG(LOGLORAW) printf(" (0x%0.2X Byte) ", (unsigned char)rxdlen);
			if((currentMode & OPMODE_RX) == OPMODE_RX){ // not in RX_SINGLE
				// In a RXCont session an RXDone has another meaning
				// Have to wait for Payload RX complete status
				BHLOG(LOGLORAW) printf("RXContWait(%dms) ", (unsigned char)rxdlen);
				delay(rxdlen);	// wait for each received FIFO-Byte (assumed Ts=1ms)  
			}

			flags = readReg(LORARegIrqFlags);	// ReRead flags for Payload-CRC error check
			if((flags & IRQ_LORA_CRCERR_MASK) == IRQ_LORA_CRCERR_MASK){
				writeReg(LORARegIrqFlags, IRQ_LORA_CRCERR_MASK); // clear CRCErr IRQ flag
				BHLOG(LOGLORAW) printf(" CRCError -> ignore IRQ\n");
				// ToDO How to tell user about this case ?
//				opmode(OPMODE_STANDBY); // Force Idle Mode -> results in recfg in main loop to RXCont

			}else{ // valid payload expected
	            // read rx quality parameters SNR & RSSI
				long int PSNR;
				long int PRSSI;

				byte value = readReg(LORARegPktSnrValue);
				if( value & 0x80 ){ // The SNR sign bit is 1
					value = ( ( ~value + 1 ) & 0xFF ) >> 2; // Invert and divide by 4
					PSNR = -value;
				} else {
					PSNR = ( value & 0xFF ) >> 2;           // Divide by 4
				}

				int16_t rssi = readReg(LORARegPktRssiValue);
				// get dB adjustment for RSSI values
                if( PSNR < 0 ){
					if( freq > RF_MID_BAND_THRESH ){
						PRSSI = RSSI_OFFSET_HF + rssi + ( rssi >> 4 ) + PSNR;
					}else{
						PRSSI = RSSI_OFFSET_LF + rssi + ( rssi >> 4 ) + PSNR;
					}
				}else{
					if( freq > RF_MID_BAND_THRESH ){
						PRSSI = RSSI_OFFSET_HF + rssi + ( rssi >> 4 );
					}else{
						PRSSI = RSSI_OFFSET_LF + rssi + ( rssi >> 4 );
					}
				}

				BHLOG(LOGLORAW) printf("\n    PRSSI:%i, RSSI:%li, ", rssi, PRSSI);
				BHLOG(LOGLORAW) printf("SNR: %li, OPMode(Reg:0x%02X) %s\n", 
							PSNR, readReg(RegOpMode),rxloraOMstring[currentMode & OPMODE_MASK]);

				// Now we can fetch the received payload from FiFo to given buffer
				// set FIFO read address pointer
		        writeReg(LORARegFifoAddrPtr, readReg(LORARegFifoRxCurrentAddr)); 

	  		    // Package size in range of BeeIoT WAN definitions ?
				if (rxdlen < BIoT_HDRLEN || rxdlen > MAX_PAYLOAD_LENGTH) {
					// Non BeeIoT Package -> store for future use
					readBuf(RegFifo, (byte *) rxframe, rxdlen);				
					BHLOG(LOGLORAW) printf("  IRQ%d: New RXPkg size out of range: %iBy -> ignored\n", (unsigned char)dio, (int) rxdlen);
//					hexdump((byte *) & rxframe, (byte) rxdlen);
					rxdlen = 0;	// got all data
					// ToDO: further processing of this proprietary message ?
					// by now ignored...
//					opmode(OPMODE_STANDBY); // the radio should have received Standby Mode

				}else{ // seems to be a valid BeeIoT package, put it to RXQueue
					BHLOG(LOGLORAR) printf("  IRQ%d: Get BeeIoT RXDataPkg[%i] - len=%iBy\n",(unsigned char)dio, (int)RXPkgIsrIdx, (int)rxdlen);
					readBuf(RegFifo, (byte *) & MyRXData[RXPkgIsrIdx], (byte) rxdlen);
//					BHLOG(LOGLORAR) hexdump((byte *) & MyRXData[RXPkgIsrIdx], (byte) rxdlen);

					 // Correct RXQueue ISR-Write pointer to next free queue buffer  
					if(++RXPkgIsrIdx >= MAXRXPKG){  // no next free RX buffer left in sequential queue round robin
						BHLOG(LOGLORAR) printf("  IRQ%d: RX Queue end reached > back to IsrIdx:0\n",(unsigned char)dio);
						RXPkgIsrIdx=0;  // wrap around
					}
//					opmode(OPMODE_STANDBY); // Force Idle Mode

					// Signal to Userland: New Entry in RX Queue
					BeeIotRXFlag++;		// polled by Main() loop
					cp_nb_rx_rcv++;		// statistics for BIoTApp(): # received pkgs.

				} // RD BeeIoT Pkg
			}// CRC check
		// RXTOUT
        } else if((flags & IRQ_LORA_RXTOUT_MASK) == IRQ_LORA_RXTOUT_MASK){
            // indicate timeout
            rxdlen = 0;
			writeReg(LORARegIrqFlags, IRQ_LORA_RXTOUT_MASK);		// clear RXTOUT IRQ flag
			rxtime = tstamp;
			BHLOG(LOGLORAR) printf(" RXTout\n");
//			opmode(OPMODE_STANDBY); // Force Idle Mode
			// ToDO How to tell user about this case ?

		} // end of IRQ validation chain

        // clear/ack all radio IRQ flags and close masking
        writeReg(LORARegIrqFlagsMask, 0xFF);   // mask all radio IRQs
        writeReg(LORARegIrqFlags, 0xFF);        // clear radio IRQ flags

    } else { // FSK modem IRQ -> should never happen
        struct timeval now;
        gettimeofday(&now, 0);
        tstamp = (uint32_t)(now.tv_sec*1000000 + now.tv_usec);

		BHLOG(LOGLORAW) printf("IRQ<%ul>: FSK-IRQ%d - should never happen (2)(OPMode: 0x%0.2X)\n", 
					(unsigned long)tstamp, (unsigned char)dio, (unsigned char) readReg(RegOpMode));

        // clear/ack all radio IRQ flags and close masking
//        writeReg(LORARegIrqFlagsMask, 0xFF);   // mask all radio IRQs
//        writeReg(LORARegIrqFlags, 0xFF);        // clear radio IRQ flags

		// Force LoRa Mode for this ISR session just to be sure
		BHLOG(LOGLORAR) printf("IRQ%d: Force ModemType from FSK to last known LoRa-Mode 0x%02X!\n",
				(unsigned char)dio, (unsigned char)currentMode);

		opmodeLora();	// preset OPMode to Lora + HF On + no FKS reg access + SLEEP
		// SetupLoRa(); // may be this is also needed ?
		// ALternatively:
//		opmode(OPMODE_STANDBY); // the radio should have received Standby Mode
    } // FSK IRQ

	opmode(OPMODE_SLEEP); // Force SLEEP Mode
	fflush(stdout);
} // end of IRQ ISR




static void MyIRQ0(void) {
  BHLOG(LOGLORAR) printf("IRQ at DIO 0 (level %i)\n", (unsigned char) irqlevel);
  if (irqlevel==0) {
	hal_disableIRQs();
	myradio_irq_handler(0);			// instead receivepacket();
	hal_enableIRQs();
	return;
  }
}

static void MyIRQ1(void) {
  BHLOG(LOGLORAR) printf("IRQ at DIO 1 (level %i)\n", (unsigned char) irqlevel);
  if (irqlevel==0){
	hal_disableIRQs();
    myradio_irq_handler(1);
	hal_enableIRQs();
  }
}

static void MyIRQ2(void) {
  BHLOG(LOGLORAR) printf("IRQ at DIO 2 (level %i)\n", (unsigned char) irqlevel);
  if (irqlevel==0){
	hal_disableIRQs();
    myradio_irq_handler(2);
	hal_enableIRQs();
  }
}

void myisr_init() {
	static int spifd;
//    hal_io_init();
    // configure radio SPI
//    hal_spi_init();	// spifd = wiringPiSPISetup(0, 10000000);
    // configure timer and interrupt handler
	
	// Mask all IRQs but RxDone for RXCONT Mode
	// We us RXCont Mode: Do we need RX_Headr as well ? RXDone should be enough
	// in RXonly mode we would have to add: IRQ_LORA_HEADER_MASK
	writeReg(LORARegIrqFlagsMask, ~(IRQ_LORA_RXDONE_MASK|IRQ_LORA_CRCERR_MASK)); // for RXDone

	//    hal_time_init();

	// Assign ISR to rising edge of DIO0 IRQ
    wiringPiISR(LORAdio0, INT_EDGE_RISING, MyIRQ0);
    wiringPiISR(LORAdio1, INT_EDGE_RISING, MyIRQ1);
    wiringPiISR(LORAdio2, INT_EDGE_RISING, MyIRQ2);  
    BHLOG(LOGLORAR) printf("  ISR_Init(): --- ISR on DIO0+1+2 assigned ---\n");

	irqlevel =0;		// enable IRQs: just to be sure...
}


void hal_disableIRQs () {
	if(++irqlevel > 0){
//    printf("    Disabled IRQs(%d)\n", irqlevel);
	}
}

void hal_enableIRQs () {
    if(--irqlevel == 0){
//      printf("    Enabled IRQs !\n");
	}
}
