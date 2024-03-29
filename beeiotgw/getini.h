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

	// GW HW components enabler flags
	int		hc_lora;		// =1 GW Lora Module enabled on client side
	int		hc_lorawan;     // =0 BIoTWAN; =1 LoRaWAN protocol on both sides
	int		hc_wifi;        // =1 GW WIFI enabled on client side
	int		hc_gps;			// =1 GW GPS module connected on client side
	int		hc_locweb;		// =1 GW report to local webpage at BEEIOTWEB
	int		hc_remweb;		// =1 GW xfer of logfiles to remote Web

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
	int		nd1_hwconfig;	// Node HW component flag field
	
	int		nd2_gwid;		// LoRa-Client2 GWID
	int		nd2_mid;		// LoRa Client2 Modem assigned to GWID
	int		nd2_appeui;		// LoRa-Client2 APP-EUI
	ulong	nd2_deveuiup;	// LoRa-Client2 DEV-EUI upper
	ulong	nd2_deveuilo;	// LoRa-Client2 DEV-EUI lower
	int		nd2_freport;	// LoRa Client2 Status Report Frq.in Min.
	int		nd2_wcalib;		// Weight cell calibration
	int		nd2_hwconfig;	// Node HW component flag field

	int		nd3_gwid;		// LoRa-Client3 GWID
	int		nd3_mid;		// LoRa Client3 Modem assigned to GWID
	int		nd3_appeui;		// LoRa-Client3 APP-EUI
	ulong	nd3_deveuiup;	// LoRa-Client3 DEV-EUI upper
	ulong	nd3_deveuilo;	// LoRa-Client3 DEV-EUI lower
	int		nd3_freport;	// LoRa Client3 Status Report Frq.in Min.
	int		nd3_wcalib;		// Weight cell calibration
	int		nd3_hwconfig;	// Node HW component flag field

	int		nd4_gwid;		// LoRa-Client4 GWID
	int		nd4_mid;		// LoRa Client4 Modem assigned to GWID
	int		nd4_appeui;		// LoRa-Client4 APP-EUI
	ulong	nd4_deveuiup;	// LoRa-Client4 DEV-EUI upper
	ulong	nd4_deveuilo;	// LoRa-Client4 DEV-EUI lower
	int		nd4_freport;	// LoRa Client4 Status Report Frq.in Min.
	int		nd4_wcalib;		// Weight cell calibration
	int		nd4_hwconfig;	// Node HW component flag field

	int		nd5_gwid;		// LoRa-Client5 GWID
	int		nd5_mid;		// LoRa Client5 Modem assigned to GWID
	int		nd5_appeui;		// LoRa-Client5 APP-EUI
	ulong	nd5_deveuiup;	// LoRa-Client5 DEV-EUI upper
	ulong	nd5_deveuilo;	// LoRa-Client5 DEV-EUI lower
	int		nd5_freport;	// LoRa Client5 Status Report Frq.in Min.
	int		nd5_wcalib;		// Weight cell calibration
	int		nd5_hwconfig;	// Node HW component flag field

	int		nd6_gwid;		// LoRa-Client6 GWID
	int		nd6_mid;		// LoRa Client6 Modem assigned to GWID
	int		nd6_appeui;		// LoRa-Client6 APP-EUI
	ulong	nd6_deveuiup;	// LoRa-Client6 DEV-EUI upper
	ulong	nd6_deveuilo;	// LoRa-Client6 DEV-EUI lower
	int		nd6_freport;	// LoRa Client6 Status Report Frq.in Min.
	int		nd6_wcalib;		// Weight cell calibration
	int		nd6_hwconfig;	// Node HW component flag field

	int		nd7_gwid;		// LoRa-Client7 GWID
	int		nd7_mid;		// LoRa Client7 Modem assigned to GWID
	int		nd7_appeui;		// LoRa-Client7 APP-EUI
	ulong	nd7_deveuiup;	// LoRa-Client7 DEV-EUI upper
	ulong	nd7_deveuilo;	// LoRa-Client7 DEV-EUI lower
	int		nd7_freport;	// LoRa Client7 Status Report Frq.in Min.
	int		nd7_wcalib;		// Weight cell calibration
	int		nd7_hwconfig;	// Node HW component flag field
	
	int		nd8_gwid;		// LoRa-Client8 GWID
	int		nd8_mid;		// LoRa Client8 Modem assigned to GWID
	int		nd8_appeui;		// LoRa-Client8 APP-EUI
	ulong	nd8_deveuiup;	// LoRa-Client8 DEV-EUI upper
	ulong	nd8_deveuilo;	// LoRa-Client8 DEV-EUI lower
	int		nd8_freport;	// LoRa Client8 Status Report Frq.in Min.
	int		nd8_wcalib;		// Weight cell calibration
	int		nd8_hwconfig;	// Node HW component flag field

	int		nd9_gwid;		// LoRa-Client9 GWID
	int		nd9_mid;		// LoRa Client9 Modem assigned to GWID
	int		nd9_appeui;		// LoRa-Client9 APP-EUI
	ulong	nd9_deveuiup;	// LoRa-Client9 DEV-EUI upper
	ulong	nd9_deveuilo;	// LoRa-Client9 DEV-EUI lower
	int		nd9_freport;	// LoRa Client9 Status Report Frq.in Min.
	int		nd9_wcalib;		// Weight cell calibration
	int		nd9_hwconfig;	// Node HW component flag field

	int		nd10_gwid;		// LoRa-Client10 GWID
	int		nd10_mid;		// LoRa Client10 Modem assigned to GWID
	int		nd10_appeui;	// LoRa-Client10 APP-EUI
	ulong	nd10_deveuiup;	// LoRa-Client10 DEV-EUI upper
	ulong	nd10_deveuilo;	// LoRa-Client10 DEV-EUI lower
	int		nd10_freport;	// LoRa Client10 Status Report Frq.in Min.
	int		nd10_wcalib;	// Weight cell calibration
	int		nd10_hwconfig;	// Node HW component flag field

	int		nd11_gwid;		// LoRa-Client11 GWID
	int		nd11_mid;		// LoRa Client11 Modem assigned to GWID
	int		nd11_appeui;	// LoRa-Client11 APP-EUI
	ulong	nd11_deveuiup;	// LoRa-Client11 DEV-EUI upper
	ulong	nd11_deveuilo;	// LoRa-Client11 DEV-EUI lower
	int		nd11_freport;	// LoRa Client11 Status Report Frq.in Min.
	int		nd11_wcalib;	// Weight cell calibration
	int		nd11_hwconfig;	// Node HW component flag field

	int		nd12_gwid;		// LoRa-Client12 GWID
	int		nd12_mid;		// LoRa Client12 Modem assigned to GWID
	int		nd12_appeui;	// LoRa-Client12 APP-EUI
	ulong	nd12_deveuiup;	// LoRa-Client12 DEV-EUI upper
	ulong	nd12_deveuilo;	// LoRa-Client12 DEV-EUI lower
	int		nd12_freport;	// LoRa Client12 Status Report Frq.in Min.
	int		nd12_wcalib;	// Weight cell calibration
	int		nd12_hwconfig;	// Node HW component flag field

	int		nd13_gwid;		// LoRa-Client13 GWID
	int		nd13_mid;		// LoRa Client13 Modem assigned to GWID
	int		nd13_appeui;	// LoRa-Client13 APP-EUI
	ulong	nd13_deveuiup;	// LoRa-Client13 DEV-EUI upper
	ulong	nd13_deveuilo;	// LoRa-Client13 DEV-EUI lower
	int		nd13_freport;	// LoRa Client13 Status Report Frq.in Min.
	int		nd13_wcalib;	// Weight cell calibration
	int		nd13_hwconfig;	// Node HW component flag field

	int		nd14_gwid;		// LoRa-Client14 GWID
	int		nd14_mid;		// LoRa Client14 Modem assigned to GWID
	int		nd14_appeui;	// LoRa-Client14 APP-EUI
	ulong	nd14_deveuiup;	// LoRa-Client14 DEV-EUI upper
	ulong	nd14_deveuilo;	// LoRa-Client14 DEV-EUI lower
	int		nd14_freport;	// LoRa Client14 Status Report Frq.in Min.
	int		nd14_wcalib;	// Weight cell calibration
	int		nd14_hwconfig;	// Node HW component flag field

	int		nd15_gwid;		// LoRa-Client15 GWID
	int		nd15_mid;		// LoRa Client15 Modem assigned to GWID
	int		nd15_appeui;	// LoRa-Client15 APP-EUI
	ulong	nd15_deveuiup;	// LoRa-Client15 DEV-EUI upper
	ulong	nd15_deveuilo;	// LoRa-Client15 DEV-EUI lower
	int		nd15_freport;	// LoRa Client15 Status Report Frq.in Min.
	int		nd15_wcalib;	// Weight cell calibration
	int		nd15_hwconfig;	// Node HW component flag field

	// section BeeIoT
	int		biot_loopwait;
	char	biot_home[PATHLEN];
	char	biot_LOGFILE[LOGFILELEN];
	char	biot_CSVFILE[LOGFILELEN];
	char	biot_CSVDAYS[LOGFILELEN];
	int		biot_actionIOfailure;
	int		biot_verbose;
	char	biot_CMDFILE[LOGFILELEN];
	char	biot_RESULTFILE[LOGFILELEN];
        
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

