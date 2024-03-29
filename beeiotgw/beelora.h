//*******************************************************************
// File:	beelora.h  
// Project: https://github.com/mchresse/BeeIoTGW
// Author:	MCHREsse
// Created on 28. Dec 2019
//
// Description:
// Semtech SX127x chip register & Lora package flow definitions
//
//----------------------------------------------------------
// Copyright (c) 2019-present, Randolph Esser
// All rights reserved.
// This file is distributed under the BSD-3-Clause License
// The complete license agreement can be obtained at: 
//     https://github.com/mchresse/BeeIoTGW/license
// For used 3rd party open source see also Readme_OpenSource.txt
//*******************************************************************

#ifndef BEELORA_H
#define BEELORA_H

using namespace std;

// Target platform types
typedef unsigned char      bit_t;
typedef unsigned char	   byte;
typedef unsigned char      u1_t;
typedef   signed char      s1_t;
typedef unsigned short     u2_t;
typedef          short     s2_t;
typedef unsigned int       u4_t;
typedef          int       s4_t;
typedef unsigned long long u8_t;
typedef          long long s8_t;
typedef const char*        str_t;
typedef              u1_t* xref2u1_t;
typedef        const u1_t* xref2cu1_t;

typedef              s4_t  ostime_t;		// tome stamp for LoRa messages

// Dragino RPi Hat with SX127x at Raspberry 40p connection: 
// -> wPi Mode !
#define CFG_sx1276_radio		// type of Dragino Hat LoRa chip

// #define RXled	LORAdio0
#define SPI0		0			// use RPi SPI channel 0
#define SPIFRQ		500000		// use SPI channel Frequency 0,5MHz

// purpose of receive window - like lmic_t.rxState
enum { RADIO_RST=0, RADIO_TX, RADIO_RX, RADIO_RXON };
enum { RXMODE_SINGLE, RXMODE_SCAN, RXMODE_RSSI };

#define MAXNODES	16			// max. # of supported/known nodes in WLTAB[] & NDB[]
// NODEIDBASE defined in BeeIoTWAN.h
enum {	NODEID1=NODEIDBASE+1, NODEID2, NODEID3, NODEID4, NODEID5,
		NODEID6, NODEID7, NODEID8, NODEID9, NODEID10, NODEID11, NODEID12,
		NODEID13, NODEID14, NODEID15
};	// Transfer IDs of LoRa Clients

// GWIDx (enum base) already defined in BeeIoTWAN.h
enum {	GWID0=GWIDx,	// default JOIN GW ID -> fallback if comm. loss
		GWID1=GWIDx-1,	// Transfer ID of first gateway (Data Srv.,  SDLog)
		GWID2=GWIDx-2,	// Transfer ID of second gateway (backup srv., FW upd.)
		GWID3=GWIDx-3,  // Future use
		GWID4=GWIDx-4	// Future use
};

// Max. time a TX request is waiting for TXDone
#define MAXTXTO		40			// TX TO at n x MAX_PAYLOAD_LENGTH in ms: 40 x 0x80ms = 5sec.

#ifndef RX_RAMPUP
#define RX_RAMPUP  (us2osticks(2000))
#endif
#ifndef TX_RAMPUP
#define TX_RAMPUP  (us2osticks(2000))
#endif

#ifndef OSTICKS_PER_SEC
//#define OSTICKS_PER_SEC 32768
#define OSTICKS_PER_SEC 20000
#elif OSTICKS_PER_SEC < 10000 || OSTICKS_PER_SEC > 64516
#error Illegal OSTICKS_PER_SEC - must be in range [10000:64516]. One tick must be 15.5us .. 100us long.
#endif

#if !HAS_ostick_conv
#define us2osticks(us)   ((ostime_t)( ((s8_t)(us) * OSTICKS_PER_SEC) / 1000000))
#define ms2osticks(ms)   ((ostime_t)( ((s8_t)(ms) * OSTICKS_PER_SEC)    / 1000))
#define sec2osticks(sec) ((ostime_t)( (s8_t)(sec) * OSTICKS_PER_SEC))
#define osticks2ms(os)   ((s4_t)(((os)*(s8_t)1000    ) / OSTICKS_PER_SEC))
#define osticks2us(os)   ((s4_t)(((os)*(s8_t)1000000 ) / OSTICKS_PER_SEC))
// Special versions
#define us2osticksCeil(us)  ((ostime_t)( ((s8_t)(us) * OSTICKS_PER_SEC + 999999) / 1000000))
#define us2osticksRound(us) ((ostime_t)( ((s8_t)(us) * OSTICKS_PER_SEC + 500000) / 1000000))
#define ms2osticksCeil(ms)  ((ostime_t)( ((s8_t)(ms) * OSTICKS_PER_SEC + 999) / 1000))
#define ms2osticksRound(ms) ((ostime_t)( ((s8_t)(ms) * OSTICKS_PER_SEC + 500) / 1000))
#endif

// ======================================================================
// AES support -> aes.cpp

#ifndef AES_ENC  // if AES_ENC is defined as macro all other values must be too
#define AES_ENC       0x00 
#define AES_DEC       0x01
#define AES_MIC       0x02
#define AES_CTR       0x04
#define AES_MICNOAUX  0x08
#endif
#ifndef AESkey  // if AESkey is defined as macro all other values must be too
extern long int * AESkey;
extern long int * AESaux;
#endif
#ifndef os_aes
u4_t os_aes (u1_t mode, xref2u1_t buf, u2_t len);
//unsigned long os_aes (byte mode, long * buf, int len);
#endif

#endif /* BEELORA_H */

