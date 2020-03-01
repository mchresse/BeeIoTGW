//*******************************************************************
// BeeLora.h  
// from Project https://github.com/mchresse/BeeIoT
//
// Description:
// BeeIoT-WAN - Lora package flow definitions
//
//----------------------------------------------------------
// Copyright (c) 2019-present, Randolph Esser
// All rights reserved.
// This file is distributed under the BSD-3-Clause License
// The complete license agreement can be obtained at: 
//     https://github.com/mchresse/BeeIoT/license
// For used 3rd party open source see also Readme_OpenSource.txt
//*******************************************************************

#ifndef BEELORA_H
#define BEELORA_H

#include <string>

// LoRaLogStatus() logging modes
#define LOGDYN	1
#define LOGSTAT 2
#define LOGALL	3

// Target platform as C library
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
typedef unsigned int       uint;
typedef const char*        str_t;
typedef              u1_t* xref2u1_t;
typedef        const u1_t* xref2cu1_t;

typedef              u4_t  devaddr_t;		// Device address
typedef              s4_t  ostime_t;		// tome stamp for LoRa messages

// Dragino RPi Hat with SX127x at Raspberry 40p connection: 
// -> wPi Mode !
#define CFG_sx1276_radio		// type of Dragino Hat LoRa chip
//				WirePi-Pin:
#define LORAcs		 6	// GPIO 25  Pin 22
#define LORAdio0	 7	// GPIO 4	Pin 7
#define LORAdio1	 4	// GPIO 23	Pin 16
#define LORAdio2	 5	// GPIO 24	Pin 18
#define LORArst		 0	// GPIO 0	Pin 11
#define LORAmosi	 12	// GPIO 10 MOSI Pin 19
#define	LORAmiso	 13	// GPIO 9  MISO Pin 21
#define LORAsck		 14	// GPIO 11 SCLK Pin 23 
#define RXled		 LORAdio0
#define	GPStxd		 15	// GPIO 15 TxD
#define GPSrxd		 16	// GPIO 16 RxD

// purpose of receive window - like lmic_t.rxState
enum { RADIO_RST=0, RADIO_TX, RADIO_RX, RADIO_RXON };
enum { RXMODE_SINGLE, RXMODE_SCAN, RXMODE_RSSI };

#define MAXNODES	10			// max. # of supported/known nodes
// NODEIDBASE defined in BeeIoTWAN.h
enum {	NODEID1=NODEIDBASE+1, NODEID2, NODEID3, NODEID4, NODEID5};	// Transfer ID of LoRa Client 5


#define MAXGW		2			// max. # of serving gateways in this network
// GWID1 defined in BeeIoTWAN.h
#define GWID1		0x98		// Transfer ID of this gateway (backup srv., SDLog, FW upd.)

// Max. time a TX request is waiting for TXDone
#define MAXTXTO		40			// TX TO at n x MAX_PAYLOAD_LENGTH in ms: 40 x 0x80ms = 5sec.
#define MAXRXPKG	10			// RX Queue Len: Max. number of parallel processed RX packages



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
// AES support 
// !!Keep in sync with lorabase.hpp!!

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

//******************************************************************
// For GW: TTN connectivity (optional)
//******************************************************************
// TTN Connection: TTN servers
// TODO: use host names and dns
#define TTNSERVER1  "54.72.145.119"   // The Things Network: croft.thethings.girovito.nl
#define TTNSERVER1b "54.229.214.112"  // The Things Network: croft.thethings.girovito.nl
#define TTNSERVER2  "192.168.1.10"    // local
#define TTNPORT		1700              // The port on which to send data

//******************************************************************
// Queue item for RX packages
typedef struct {
//	beeiotmsg_t * next;	// connection to next active message of node
    byte idx;           // index of sent message: 0..255 (round robin)
    byte retries;       // number of initiated retries
	byte ack;			// ack flag 1 = message received
    beeiotpkg_t * pkg; // sent message struct
} beeiotmsg_t;

// NodeWLTable -> manually preset for reference of join requests
typedef struct{
	byte	nodeid;
	byte	gwid;
	byte	AppEUI[8];	// corresponding unique AppEUI of serving App
	byte	DevEUI[8];	// Node unique DevEUI
	byte	reportfrq;	// [min] Frequency of Status data reports
	bool	joined;		// == 0 Node has not been joined; ==1 joined
} nodewltable_t;

// NodeDB[] typeset
typedef struct{
	beeiotmsg_t msg;		// root point of node message queue chain
	nodewltable_t * pwltab; // ptr to corresponding WL node table
	joinpar_t	nodeinfo;	// Node info descriptor
	devcfg_t	nodecfg;	// parameter set for node side runtime config.
	byte		AppSKey[16];
	byte		NwSKey[16];
	devaddr_t	DevAddr;
} nodedb_t;

//******************************************************************
// global function prototypes:


void sendstat();
void sendudp(char *msg, int length);

void hexdump(unsigned char * msg, int len);
void Printhex(unsigned char * pbin, int bytelen, const char * s = "0x", int format=1, int dir=0);
void Printbit(unsigned char * pbin, int bitlen,  const char * s = "0b", int format=1, int dir=0);
int ByteStreamCmp(byte * bina, byte * binb, int binlen);

// in radio.c
void startrx (u1_t rxmode, int rxtime);
void starttx (byte* frame, byte dataLen);
void os_radio (u1_t mode);
void hal_disableIRQs ();
void hal_enableIRQs ();
void opmode (u1_t mode);
void opmodeLora();
void SetupLoRa(void);
void PrintLoraStatus(int logtype);
void myisr_init(void);



#endif /* BEELORA_H */

