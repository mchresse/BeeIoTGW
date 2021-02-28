/*
* The "getini.h" library is distributed under the New BSD license:
*
* Copyright (c) 2020, Randolph Esser
* All rights reserved.
*
* Copyright (c) 2009, Ben Hoyt
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above copyright
*      notice, this list of conditions and the following disclaimer in the
*      documentation and/or other materials provided with the distribution.
*    * Neither the name of Ben Hoyt nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY BEN HOYT ''AS IS'' AND ANY
* EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL BEN HOYT BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* --- end of LICENSE ---
*
* getini.h is released under the New BSD license. Go to the project
* home page for more info: https://github.com/mchresse/beeiotgw
*/
/* 
 * File:   getini.h
 * Created on 01.02.2020
 */

#ifndef GETINI_H
#define GETINI_H

#ifdef __cplusplus
extern "C" {
#endif

	
// Config file name
#define CONFIGINI	"./config.ini"
#define CONFIGINI2	"./config.ini~"
#define LOGFILELEN	255		// length of LOG file NAME
#define VERSIONLEN	10		// length of Config.ini version string
#define PATHLEN		1024	// length of Home path string

// structure for config ini file parameters parsed by getini()
typedef struct{
    unsigned char version[VERSIONLEN];			// version of ini file
	int		vmajor;			// BIoTWAN Version
	int		vminor;

	// component enabler flags
	int		hc_lora;
	int		hc_lorawan;     // =0 BIoTWAN; =1 LoRaWAN
	int		hc_wifi;        // =1 WIFI enabled on client side
	int		hc_gps;
	int		hc_locweb;		// report to local webpage at BEEIOTWEB
	int		hc_remweb;

	// section HWCONFIG
	int		loranumchn;		// number of support BIoT LoRa Channels
	int		loradefchn;		// Default channel number for JOIN requests

	// LoRA Port 0 GPIO connection 	
	int		loraxmiso;		// SPI0 MISO
	int		loraxmosi;		// SPI0 MOSI
	int		loraxclk;		// SPI0 SCLK

	int		lora_cs0;		// Lora CS0
	int		lora0rst;		// Lora RST
	int		lora0dio0;		// Lora DIO0
	int		lora0dio1;		// Lora DIO1
	int		lora0dio2;		// Lora DIO2
	int		lora0dio3;		// Lora DIO3
	int		lora0dio4;		// Lora DIO4
	int		lora0dio5;		// Lora DIO5
	int		lora0channel;	// Lora channel cfg

	int		lora_cs1;		// Lora CS1
	int		lora1rst;		// Lora RST
	int		lora1dio0;		// Lora DIO0
	int		lora1dio1;		// Lora DIO1
	int		lora1dio2;		// Lora DIO2
	int		lora1dio3;		// Lora DIO3
	int		lora1dio4;		// Lora DIO4
	int		lora1dio5;		// Lora DIO5
	int		lora1channel;	// Lora channel cfg
	int		gps0RX;			// GPS  RX
	int		gps0TX;			// GPS  TX

	// Node Registration table
	int		nd1_gwid;		// LoRa-Client1 GWID
	int		nd1_mid;		// LoRa Client1 Modem assigned to GWID
	int		nd1_appeui;		// LoRa-Client1 APP-EUI
	ulong	nd1_deveuiup;	// LoRa-Client1 DEV-EUI upper
	ulong	nd1_deveuilo;	// LoRa-Client1 DEV-EUI lower
	int		nd1_freport;	// LoRa Client1 Status Report Frq.in Min.
	int		nd1_wcalib;		// Weight cell calibration

	int		nd2_gwid;		// LoRa-Client2 GWID
	int		nd2_mid;		// LoRa Client2 Modem assigned to GWID
	int		nd2_appeui;		// LoRa-Client2 APP-EUI
	ulong	nd2_deveuiup;	// LoRa-Client2 DEV-EUI upper
	ulong	nd2_deveuilo;	// LoRa-Client2 DEV-EUI lower
	int		nd2_freport;	// LoRa Client2 Status Report Frq.in Min.
	int		nd2_wcalib;		// Weight cell calibration

	int		nd3_gwid;		// LoRa-Client3 GWID
	int		nd3_mid;		// LoRa Client3 Modem assigned to GWID
	int		nd3_appeui;		// LoRa-Client3 APP-EUI
	ulong	nd3_deveuiup;	// LoRa-Client3 DEV-EUI upper
	ulong	nd3_deveuilo;	// LoRa-Client3 DEV-EUI lower
	int		nd3_freport;	// LoRa Client3 Status Report Frq.in Min.
	int		nd3_wcalib;		// Weight cell calibration

	int		nd4_gwid;		// LoRa-Client4 GWID
	int		nd4_mid;		// LoRa Client4 Modem assigned to GWID
	int		nd4_appeui;		// LoRa-Client4 APP-EUI
	ulong	nd4_deveuiup;	// LoRa-Client4 DEV-EUI upper
	ulong	nd4_deveuilo;	// LoRa-Client4 DEV-EUI lower
	int		nd4_freport;	// LoRa Client4 Status Report Frq.in Min.
	int		nd4_wcalib;		// Weight cell calibration
	
	// section BeeIoT
	int		biot_loopwait;
	char	biot_home[PATHLEN];
	char	biot_LOGFILE[LOGFILELEN];
	char	biot_CSVFILE[LOGFILELEN];
	char	biot_CSVDAYS[LOGFILELEN];
	int		biot_actionIOfailure;
	int		biot_verbose;
        
    // section WIFI
    char	wifi_ssid[32];                     // Server/client WIFI SSID
    char	wifi_key[32];                      // Server/client WIFI PWD
        
	// section WEBUI
	int		web_autoupdate;                 // =0 disabled remote Web Space update via FTP
	char	web_root[PATHLEN];
	char	web_beekeeper[LOGFILELEN];
	char	web_locdat1[LOGFILELEN];
	char	web_locdat2[LOGFILELEN];
	char	web_locdat3[LOGFILELEN];
	int		web_locplz;					// PLZ for weather data from web
	char	web_picsmall[LOGFILELEN];
	char	web_piclarge[LOGFILELEN];
	char	web_noticefile[LOGFILELEN];
	char	web_deffile[LOGFILELEN];	// default web page file for update
	int		web_alarm_on;
	int		web_alarm_weight;
	int		web_alarm_swarm;
	int		web_alarm_batt1;			// =0 disabled; 1..100% enabled, typical 100%
	int		web_alarm_batt2;			// =0 disabled; 1..100% enabled, typical 0%

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

configuration * getini(char* inifile);
configuration * newini(char* inifile);
int setini(char *inifile, char * inifilebuf, char * section, char * name, char * insertline);

#ifdef __cplusplus
}
#endif

#endif /* GETINI_H */

