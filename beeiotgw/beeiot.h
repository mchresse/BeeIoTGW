/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   beeiot.h
 * Author: MCHREsse
 *
 * Created on 19. Dezember 2019, 13:10
 */

#ifndef BEEIOT_H
#define BEEIOT_H

// typedef bool boolean;

// Definitions of LogLevel masks instead of verbose mode (for uint16_t bitfield)
#define LOGBH		1		// 1:   Behive Setup & Loop program flow control
#define LOGOW		2		// 2:   1-wire discovery and value read
#define LOGHX		4		// 4:   HX711 init & get values
#define LOGEPD		8		// 8:   ePaper init & control
#define LOGLAN		16		// 16:  Wifi init & LAN Import/export in all formats and protocols
#define LOGSD		32		// 32:  SD Card & local data handling
#define LOGADS		64  	// 64:  ADS BMS monitoring routines /w ADS1115S
#define LOGSPI		128		// 128: SPI Init
#define LOGLORAR	256		// 256: LoRa Init: Radio layer
#define LOGLORAW	512		// 512: LoRa Init: BeeIoT-WAN

#define	BHLOG(m)	if(lflags & m)	// macro for Log evaluation (type: uint)

//*******************************************************************
// Global data declarations
//*******************************************************************

#define DROWNOTELEN	129
#define LENTMSTAMP	20
typedef struct {				// data elements of one log line entry
    int     index;				// index of status data data assigned by the node
    char  	timeStamp[LENTMSTAMP]; // time stamp of sensor row  e.g. 'YYYY-MM-DD HH:MM:SS'
	float	HiveWeight;			// weight in [kg]
	float	TempExtern;			// external temperature
	float	TempIntern;			// internal temp. of weight cell (for compensation)
	float	TempHive;			// internal temp. of bee hive
	float	TempRTC;			// internal temp. of RTC module
	float	ESP3V;				// ESP32 DevKit 3300 mV voltage level
	float	Board5V;			// Board 5000 mV Power line voltage level
	float	BattCharge;			// Battery Charge voltage input >0 ... 5000mV 
	float	BattLoad;			// Battery voltage level (typ. 3700 mV)
	int		BattLevel;			// Battery Level in % (3200mV (0%) - 4150mV (100%))
	char	comment[DROWNOTELEN];
} datrow;

#define datasetsize	2			// max. # of local dataset buffer: each for "looptime" seconds
#define LENFDATE 	21
#define LENDATE		11
#define LENTIME		9
#define LENIPADDR	16
#define LENMACADDR	18
typedef struct {
	// save timestamp of last datarow entry:
    char	formattedDate[LENFDATE]; // Variable to save date and time; 2019-10-28T08:09:47Z
    char	date[LENDATE];		// Date Stamp: 2019-10-28
    char	time[LENTIME];		// Time Stamp: 08:10:15
    char	ipaddr[LENIPADDR];	// local IPv4 Address xxx:xxx:xxx:xxx
    char	macaddr[LENMACADDR];// local MAC  Address xx:xx:xx:xx:xx:xx
    int     loopid;             // sensor data set sample ID assigned by the node
	int		packid;				// package index to assure LoRa sequentiality
    uint64_t  BoardID;          // unique Identifier of MUC board (6 Byte effective) not used by now
	datrow	dlog[datasetsize];	// all sensor data set logs till upload to server
} dataset;

#define	CSV_HEADER		"Datum,Zeit,GewichtBeute,Temp.Extern,TempIntern,Temp.Beute1,Temp.RTC,Batt.ESP3V,Board5V,BattCharge,BattLoad,BattLevel"
#define LOGLINELEN		1024	// max length of one log textline incl. comments
#define LOGDATELEN		32		// length of log dat&time string
#define LOGNOTELEN		512		// length of logline comment string
#define WEBIDXMAXLLEN	512
#define WEBNOTICEMARKER "//NoticeAddHere"
	
int	beelog(char * comment);
int beecsv(dataset  * data);

// Config file name
#define CONFIGINI	"./config.ini"
#define CONFIGINI2	"./config.ini~"
#define LOGFILELEN	255		// length of LOG file NAME
#define VERSIONLEN	10		// length of Config.ini version string
#define PATHLEN		1024	// length of Home path string

// structure for config ini file parameters parsed by getini()
typedef struct{
    unsigned char version[VERSIONLEN];			// version of ini file
	// section HWCONFIG
	int		loramiso;		// SPI0 MISO
	int		loramosi;		// SPI0 MOSI
	int		loraclk;		// SPI0 SCLK
	int		loracs;			// SPI0 CS\
	int		gpsRX;			// GPS  RX
	int		gpsTX;			// GPS  TX
	int		lorarst;		// Lora RST\
	int		loradio0;		// Lora DIO0
	int		loradio1;		// Lora DIO1
	int		loradio2;		// Lora DIO2
	int		loradio3;		// Lora DIO3
	int		loradio4;		// Lora DIO4
	int		loradio5;		// Lora DIO5
	// component enabler flags
	int		hc_lora;
	int		hc_lorawan;
	int		hc_gpsmouse;
	int     hc_webui;		// webpage at BEELOGWEB
	// section BeeLog
	int		bh_loopwait;
	char	bh_home[PATHLEN];
	char	bh_LOGFILE[LOGFILELEN];
	char	bh_CSVFILE[LOGFILELEN];
	char	bh_CSVDAYS[LOGFILELEN];
	int		bh_actionIOfailure;
	int		bh_verbose;
	// section OneWire
	float	owtcint;			// temp. correction internal sensor
	float	owtcext;			// temp. compensation external sensor
	float	owtchive1;			// temp. compensation hive sensor
	// section WEBUI
	char	web_root[PATHLEN];
	char	web_beekeeper[LOGFILELEN];
	char	web_locdat1[LOGFILELEN];
	char	web_locdat2[LOGFILELEN];
	char	web_locdat3[LOGFILELEN];
	int		web_locplz;				// PLZ for weather data from web
	char	web_picsmall[LOGFILELEN];
	char	web_piclarge[LOGFILELEN];
	char	web_noticefile[LOGFILELEN];
	int		web_autoupdate;
	char	web_deffile[LOGFILELEN];// default web page file for update
	int		web_alarm_on;
	int		web_alarm_weight;
	int		web_alarm_swarm;
	int		web_alarm_batt;			// =0 disabled; 1..99% enabled, typical 98%
	// section EXPORT
	char	exp_ftpurl[PATHLEN];
	char	exp_ftpport[10];
	char	exp_ftppath[PATHLEN];
	char	exp_ftpproxy[PATHLEN];
	char	exp_ftpproxyport[10];
	char	exp_ftpuser[LOGFILELEN];
	char	exp_bkuppath[PATHLEN];
	char	exp_bkupfile[LOGFILELEN];
} configuration;

#endif /* BEEIOT_H */

