//*******************************************************************
// Beeiot.h  
// from Project https://github.com/mchresse/BeeIoT
// Author: MCHREsse
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

#ifndef BEEIOT_H
#define BEEIOT_H

#include "getini.h"

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
#define	EX_NWSRV_INIT	0x0301		// NwSrv Constructor failed -> del. instance
#define	EX_JSRV_INIT	0x0401		// JoinSrv Constructor failed -> del. instance
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
	int	BattLevel;		// Battery Level in % (3200mV (0%) - 4150mV (100%))
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
	
int	beelog(char *   comment);
int beecsv(dataset  * data);


#endif /* BEEIOT_H */

