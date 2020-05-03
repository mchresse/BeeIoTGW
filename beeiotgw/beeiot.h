//*******************************************************************
// File:	Beeiot.h  
// Project: https://github.com/mchresse/BeeIoTGW
// Author:	MCHREsse
// Created on 04. Dec 2019
//
// Description:
// BeeIoT-GateWay - Main program logging & data warehouse definitionS
//
//----------------------------------------------------------
// Copyright (c) 2019-present, Randolph Esser
// All rights reserved.
// This file is distributed under the BSD-3-Clause License
// The complete license agreement can be obtained at: 
//     https://github.com/mchresse/BeeIoTGW/license
// For used 3rd party open source see also Readme_OpenSource.txt
//*******************************************************************
#ifndef BEEIOT_H
#define BEEIOT_H
using namespace std;

#include "getini.h"

#define MAXGW		2			// max. # of serving gateways in this network

// Definitions of LogLevel masks instead of verbose mode (for uint16_t bitfield)
#define LOGBH		1		// 1:   Behive Setup & Loop program flow control
#define LOGOW		2		// 2:   1-wire discovery and value read
#define LOGHX		4		// 4:   HX711 init & get values
#define LOGEPD		8		// 8:   ePaper init & control
#define LOGLAN		16		// 16:  Wifi init & LAN Import/export in all formats and protocols
#define LOGSD		32		// 32:  SD Card & local data handling
#define LOGADS		64              // 64:  ADS BMS monitoring routines /w ADS1115S
#define LOGSPI		128		// 128: SPI Init
#define LOGLORAR	256		// 256: LoRa Init: Radio class
#define LOGLORAW	512		// 512: LoRa Init: BeeIoT-WAN (NwSrv class)
#define LOGQUE		1024	//1024: MsgQueue & MsgBuffer class handling
#define LOGJOIN		2048	//2048: JOIN service class
#define LOGBIOT		4096	//4096: BIoT	Application class
#define LOGGH		8192	//8192: GH		Application class
#define LOGTURTLE  16384	//16384:Turtle	Application class

#define	BHLOG(m)	if(lflags & m)	// macro for Log evaluation (type: uint)

// Class Exception return codes
// MSB declares Base of Except. Class (like 0x10xx:	Exceptions from Radio class)
// LSB declares Except. Type	(like 0x01: Constructor failed
#define	EX_RADIO_INIT	0x0101		// Radio Constructor failed -> del. instance
#define	EX_MSGQU_INIT	0x0201		// MsgQueue Constructor failed -> del. instance
#define	EX_NWSRV_INIT1	0x0301		// NwSrv Constructor failed -> wrong Input values
#define	EX_NWSRV_INIT2	0x0302		// NwSrv Constructor failed -> WiringPiSPI failed
#define	EX_NWSRV_INIT3	0x0303		// NwSrv Constructor failed -> No active modem left
#define	EX_JSRV_INIT	0x0401		// JoinSrv Constructor failed -> del. instance
#define	EX_APPS_INIT	0x1000		// App: BIoTApp proxy Constructor failed -> del. instance
#define	EX_APPBIOT_INIT	0x1001		// App: BIoT Srv Constructor failed -> del. instance
#define	EX_APPGH_INIT	0x1101		// App: GH Srv Constructor failed -> del. instance
#define	EX_APPTURTLE_INIT 0x1201	// App: Turtle Srv Constructor failed -> del. instance

// Action on IO failure (e.g. file open failure, No OW sensors, No weightcell)
// config -> bh_actionIOfailure
#define AOF_NOOP	0               // ignore: continue or retry
#define AOF_EXIT	1               // stop program with exit code EXIT_FAILURE
#define AOF_REBOOT	2               // initiate system("reboot") -> cron restart of beehive expected

//*******************************************************************
// Global data declarations
//*******************************************************************
// Global modem HW IO configuration settings (per instance)
typedef struct {
		uint8_t	sxcs;		// Chip Select
		uint8_t	sxrst;		// Reset line
		uint8_t	sxdio0;		// DIOx IRQ line
		uint8_t	sxdio1;		// DIOx IRQ line
		uint8_t	sxdio2;		// DIOx IRQ line
		uint8_t	sxdio3;		// DIOx IRQ line
		uint8_t	sxdio4;		// DIOx IRQ line		
		uint8_t	sxdio5;		// DIOx IRQ line		
}iopins_t;

// Global Modem descriptor
typedef struct{
	uint8_t		modemid;	// index of modem instance	-> set by main() cfgini
	uint8_t		gwid;		// modem corresponding Pkg GWIDx
	uint8_t		chncfgid;	// ID of channel configuration set
	iopins_t	iopins;		// GPIO port definition	-> set by main() cfgini
}modemcfg_t;

// forward declaration for  gwbind_t
class Radio;
class MsgQueue;
class NwSrv;
class JoinSrv;
class AppSrv;

// global Gateway bind list of attached services
typedef struct {
	int			nmodemsrv;	// number of served modems after activated NwService
	int			joindef;	// default JOIN modem ID (init by cfgini.loradefchn)
	Radio		* modem[MAXGW];	// ptr to modem-instance list -> set by main.initall()
	MsgQueue	* gwq;		// ptr to modem Msg Queue for all GW channels
	NwSrv		* nws;		// ptr to the one and only Network service instance
	JoinSrv		* jsrv;		// ptr to the one and only Join Service instance
	AppSrv		* apps;		// ptr to the one and only App-Services instance
	modemcfg_t	* gwset;	// Table of GateWay/Modem related config sets for Radio Instantiation

	// Statistic: to be initialize/updated by radio layer during rx/tx package handling
    uint32_t cp_nb_rx_rcv;	// # received packages
    uint32_t cp_nb_rx_ok;	// # of correct received packages
    uint32_t cp_up_pkt_fwd;	// # of sent status packages to REST/WEb service
    uint32_t cp_nb_rx_bad;	// # of invalid RX packages
    uint32_t cp_nb_rx_crc;	// # of RX packages /w CRC error
}gwbind_t;


// Radio-Modem internal used config channel set
typedef struct{ 
	uint64_t freq;			// =EU868_F1..9,DN (EU868_F1: 868.1MHz)
	uint8_t pw;				// =2-16  TX PA Mode (14)
	sf_t	sf;				// =0..8 Spreading factor FSK,7..12,SFrFu (1:SF7)
	bw_t	bw;				// =0..3 RFU Bandwidth 125-500 (0:125kHz)
	cr_t	cr;				// =0..3 Coding mode 4/5..4/8 (0:4/5)
	uint8_t	ih;				// =1 implicite Header Mode (0)
	uint8_t ihlen;			// =0..n if IH -> header length (0)
	uint8_t nocrc;			// =0/1 no CRC check used for Pkg (0)
	uint8_t noRXIQinv;		// =0/1 flag to switch RX+TX IQinv. on/off (1)
}chncfg_t;

#define DROWNOTELEN	129
#define LENTMSTAMP	20
typedef struct {			// data elements of one log line entry
    int     index;			// index of status data data assigned by the node
    char  	timeStamp[LENTMSTAMP];  // time stamp of sensor row  e.g. 'YYYY-MM-DD HH:MM:SS'
	float	HiveWeight;		// weight in [kg]
	float	TempExtern;		// external temperature
	float	TempIntern;		// internal temp. of weight cell (for compensation)
	float	TempHive;		// internal temp. of bee hive
	float	TempRTC;		// internal temp. of RTC module
	float	ESP3V;			// ESP32 DevKit 3300 mV voltage level
	float	Board5V;		// Board 5000 mV Power line voltage level
	float	BattCharge;		// Battery Charge voltage input >0 ... 5000mV 
	float	BattLoad;		// Battery voltage level (typ. 3700 mV)
	int		BattLevel;		// Battery Level in % (3200mV (0%) - 4150mV (100%))
	char	comment[DROWNOTELEN];
} datrow;

#define datasetsize	2		// max. # of local dataset buffer: each for "looptime" seconds
#define LENFDATE 	21
#define LENDATE		11
#define LENTIME		9
#define LENIPADDR	16
#define LENMACADDR	18
typedef struct {
	// save timestamp of last datarow entry:
    char	formattedDate[LENFDATE];// Variable to save date and time; 2019-10-28T08:09:47Z
    char	date[LENDATE];		// Date Stamp: 2019-10-28
    char	time[LENTIME];		// Time Stamp: 08:10:15
    char	ipaddr[LENIPADDR];	// local IPv4 Address xxx:xxx:xxx:xxx
    char	macaddr[LENMACADDR];// local MAC  Address xx:xx:xx:xx:xx:xx
    int		loopid;             // sensor data set sample ID assigned by the node
    int		packid;				// package index to assure LoRa sequentiality
    uint64_t BoardID;			// unique Identifier of MIC board (6 Byte effective) not used by now
    uint8_t	 NodeID;			// unique Identifier (short) of BIoT Network to expand CSV file name
    datrow	dlog[datasetsize];	// all sensor data set logs till upload to server
} dataset;

#define	CSV_HEADER	"Datum,Zeit,GewichtBeute,Temp.Extern,TempIntern,Temp.Beute1,Temp.RTC,Batt.ESP3V,Board5V,BattCharge,BattLoad,BattLevel"
#define LOGLINELEN	1024	// max length of one log textline incl. comments
#define LOGDATELEN	32		// length of log dat&time string
#define LOGNOTELEN	512		// length of logline comment string
#define WEBIDXMAXLLEN	512
#define WEBNOTICEMARKER "//NoticeAddHere"
	
// Some helper functions residing in main()
void hexdump(unsigned char * msg, int len);
void Printhex(unsigned char * pbin, int bytelen, const char * s = "0x", int format=1, int dir=0);
void Printbit(unsigned char * pbin, int bitlen,  const char * s = "0b", int format=1, int dir=0);


#endif /* BEEIOT_H */

