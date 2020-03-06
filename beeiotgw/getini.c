/*
* The "inih" library is distributed under the New BSD license:
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
* ini.h is released under the New BSD license. Go to the project
* home page for more info: https://github.com/benhoyt/inih
********************************************************************************
*/

/*******************************************************************************
 * Sub Module GetINI
 * parse BeeLog CONFIG FILE (config.ini) for initial runtime parameters
 * of beehive program
 * 
 * The functions handler(), getini() and setini() are derived from "inih" project
 * and relies on Lincese conditions as listed above by Ben Hoyt (2009)
 *
 * The function newini() is added for BeeLog config.ini initialization in case
 * of absence of any ini file -> relies onder central BeeLog project license
 * conditions as defined in the main() function module by R.Esser(2019)
 *
 *******************************************************************************
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ini.h"
#include "getini.h"

configuration config;	// global default runtime data configuration buffer
						// used by all modules



static int handler(void* inibuf, const char* section, const char* name, const char* value){
configuration* pconfig = (configuration*) inibuf;
#define MATCH(s, n) strcmp(section, s) == 0 && strcmp(name, n) == 0

    if (strcmp(name, "VERSION") == 0) {
        strcpy(pconfig->version, value);
    } else if (strcmp(name, "VMAJOR") == 0) {
        pconfig->vmajor = atoi(value);
    } else if (strcmp(name, "VMINOR") == 0) {
        pconfig->vminor  = atoi(value);
        
                } else if (MATCH("HWCONFIG", "LORANUMCHN")) {
			pconfig->loranumchn = atoi(value);
                } else if (MATCH("HWCONFIG", "LORADEFCHN")) {
			pconfig->loradefchn = atoi(value);
                } else if (MATCH("HWCONFIG", "LORA0CS")) {
			pconfig->lora0cs = atoi(value);
		} else if (MATCH("HWCONFIG", "LORA0MOSI")) {
			pconfig->lora0miso = atoi(value);
		} else if (MATCH("HWCONFIG", "LORA0MISO")) {
			pconfig->lora0mosi = atoi(value);
		} else if (MATCH("HWCONFIG", "LORA0SCK")) {
			pconfig->lora0clk = atoi(value);
		} else if (MATCH("HWCONFIG", "LORA0RST")) {
			pconfig->lora0rst = atoi(value);
		} else if (MATCH("HWCONFIG", "LORA0DIO0")) {
			pconfig->lora0dio0 = atoi(value);
		} else if (MATCH("HWCONFIG", "LORA0DIO1")) {
			pconfig->lora0dio1 = atoi(value);
		} else if (MATCH("HWCONFIG", "LORA0DIO2")) {
			pconfig->lora0dio2 = atoi(value);
		} else if (MATCH("HWCONFIG", "LORA0DIO3")) {
			pconfig->lora0dio3 = atoi(value);
		} else if (MATCH("HWCONFIG", "LORA0DIO4")) {
			pconfig->lora0dio4 = atoi(value);
		} else if (MATCH("HWCONFIG", "LORA0DIO5")) {
			pconfig->lora0dio5 = atoi(value);
		} else if (MATCH("HWCONFIG", "GPS0TXD")) {
			pconfig->gps0TX = atoi(value);
		} else if (MATCH("HWCONFIG", "GPS0RXD")) {
			pconfig->gps0RX = atoi(value);

		} else if (MATCH("HWCONFIG", "HCLORA")) {
			pconfig->hc_lora = atoi(value);
		} else if (MATCH("HWCONFIG", "HCLORAWAN")) {
			pconfig->hc_lorawan = atoi(value);
		} else if (MATCH("HWCONFIG", "HCWIFICLIENT")) {
			pconfig->hc_wifi = atoi(value);
		} else if (MATCH("HWCONFIG", "HCGPS")) {
			pconfig->hc_gps = atoi(value);
		} else if (MATCH("HWCONFIG", "HCLOCWEB")) {
			pconfig->hc_locweb = atoi(value);
		} else if (MATCH("HWCONFIG", "HCREMWEB")) {
			pconfig->hc_remweb = atoi(value);
    } else 
		if (       MATCH("BEEIOT", "BHLOOPWAIT")) {
			pconfig->biot_loopwait = atoi(value);
		} else if (MATCH("BEEIOT", "BEEIOTHOME")) {
			strcpy(pconfig->biot_home,value);
		} else if (MATCH("BEEIOT", "LOGFILE")) {
			strcpy(pconfig->biot_LOGFILE,value);
		} else if (MATCH("BEEIOT", "CSVFILE")) {
			strcpy(pconfig->biot_CSVFILE,value);
		} else if (MATCH("BEEIOT", "CSVDAYS")) {
			strcpy(pconfig->biot_CSVDAYS,value);	
		} else if (MATCH("BEEIOT", "ACTIOFAIL")) {
			pconfig->biot_actionIOfailure = atoi(value);
		} else if (MATCH("BEEIOT", "VERBOSE")) {
			pconfig->biot_verbose = atoi(value);
    } else 
		if (       MATCH("HX711", "TARA")) {
			pconfig->hxtara = atol(value);
		} else if (MATCH("HX711", "TARASET")) {
			pconfig->hxtaraset = atoi(value);
		} else if (MATCH("HX711", "REFKG")) {
			pconfig->hxrefkg = atol(value);
		} else if (MATCH("HX711", "TEMPCOMP")) {
			pconfig->hxtempcomp = atof(value);
		} else if (MATCH("HX711", "NSAMPLES")) {
			pconfig->hxnsamples = atoi(value);
    } else 
		if (       MATCH("ONEWIRE", "TEMPCINT")) {
			pconfig->owtcint = atof(value);
		} else if (MATCH("ONEWIRE", "TEMPCEXT")) {
			pconfig->owtcext = atof(value);
		} else if (MATCH("ONEWIRE", "TEMPCHIVE")) {
			pconfig->owtchive = atof(value);
    } else 
		if (       MATCH("WIFI", "WIFISSID")) {
			strcpy(pconfig->wifi_ssid,value);
		} else if (MATCH("WIFI", "WIFIKEY")) {
			strcpy(pconfig->wifi_key,value);
    } else 
                if (       MATCH("WEBUI", "AUTOUPDATE")) {
			pconfig->web_autoupdate = atoi(value);
                } else if (MATCH("WEBUI", "BEEIOTWEB")) {
			strcpy(pconfig->web_root,value);
		} else if (MATCH("WEBUI", "BEEKEEPER")) {
			strcpy(pconfig->web_beekeeper,value);
		} else if (MATCH("WEBUI", "LOCDAT1")) {
			strcpy(pconfig->web_locdat1,value);
		} else if (MATCH("WEBUI", "LOCDAT2")) {
			strcpy(pconfig->web_locdat2,value);
		} else if (MATCH("WEBUI", "LOCDAT3")) {
			strcpy(pconfig->web_locdat3,value);
		} else if (MATCH("WEBUI", "LOCPLZ")) {
			pconfig->web_locplz = atoi(value);
		} else if (MATCH("WEBUI", "PICSMALL")) {
			strcpy(pconfig->web_picsmall,value);
		} else if (MATCH("WEBUI", "PICLARGE")) {
			strcpy(pconfig->web_piclarge,value);
		} else if (MATCH("WEBUI", "NOTICEFILE")) {
			strcpy(pconfig->web_noticefile,value);
		} else if (MATCH("WEBUI", "WEBDEFFILE")) {
			strcpy(pconfig->web_deffile	,value);
		} else if (MATCH("WEBUI", "ALARMON")) {
			pconfig->web_alarm_on = atoi(value);
		} else if (MATCH("WEBUI", "ALARMWEIGHT")) {
			pconfig->web_alarm_weight = atoi(value);
		} else if (MATCH("WEBUI", "ALARMSWARM")) {
			pconfig->web_alarm_swarm = atoi(value);
		} else if (MATCH("WEBUI", "ALARMBATT1")) {
			pconfig->web_alarm_batt1 = atoi(value);
		} else if (MATCH("WEBUI", "ALARMBATT2")) {
			pconfig->web_alarm_batt2 = atoi(value);
    } else 	
		if (       MATCH("EXPORT", "EXFTPURL")) {
			strcpy(pconfig->exp_ftpurl,value);
		} else if (MATCH("EXPORT", "EXFTPPORT")) {
			strcpy(pconfig->exp_ftpport,value);
		} else if (MATCH("EXPORT", "EXFTPPATH")) {
			strcpy(pconfig->exp_ftppath,value);
		} else if (MATCH("EXPORT", "EXFTPPROXY")) {
			strcpy(pconfig->exp_ftpproxy,value);
		} else if (MATCH("EXPORT", "EXFTPPROXYPORT")) {
			strcpy(pconfig->exp_ftpproxyport,value);
		} else if (MATCH("EXPORT", "EXFTPUSER")) {
			strcpy(pconfig->exp_ftpuser,value);
		} else if (MATCH("EXPORT", "BKUPPATH")) {
			strcpy(pconfig->exp_bkuppath,value);
		} else if (MATCH("EXPORT", "BKUPFILE")) {
			strcpy(pconfig->exp_bkupfile,value);
    } else{
        return 0;  // unknown section/name, skip
    }
    return 1;
}

/*******************************************************************************
* Function: getini()
* Creation: 01.11.2017
* 
* open Beehive runtime config file (as defined by inifile string) 
* and parse INIT key/value pairs into resulting configuraion struct
*
* Input :
* inifile	string of beehive config file in ini format
*
* Global:	
*	config	struct of runtime config data 'configuration'
*
* Output:
*	*config	pointer on globale config data struct: 'config'
*********************************************************************************
*/
configuration* getini(char* inifile){
	
    if (ini_parse(inifile, handler, &config) < 0) {
        printf("    GetINI: Can't load user cfgfile: %s\n", inifile);
		if(!newini(inifile)){	// creation of new config file works ?
			printf("    GetINI: Can't create new cfgfile at %s - Giving Up !\n", inifile);
			return (NULL);				// No -> we give up
		}
		printf("    GetINI: new INI File created at %s\n", inifile);
		printf("\n    Bitte %s File individuell konfigurieren vor BeeIoTGW Neustart !\n", inifile);
		exit(EXIT_FAILURE);
	}
    return (&config);
}

/*******************************************************************************
* Function: SetINI()
* Creation: 01.11.2017
* 
* Scan config.ini file of Beehive and insert given insertline at given 
* section and key-name 
* 
* Input :
* inifile		name of original config file
* inifilebuf	name of temporary copy file for expansion (just to assure its unique name)
* section		section name for insertion
* name			name of new Key, whose line is to be replaced
* insertline	new line string
*
* Global:	-
*
* Output:
*	rc		0 all done as expected
*			-1	inifile could npt be opened in read mode
*			-2	Iniflebuf could not be opened in write mode
*			-3	section name not finished by ']' -> inconsistent ini file
*			-4	key 'name' not found in inifile
*********************************************************************************
*/
int setini(char *inifile, char * inifilebuf, char * section, char * name, char * insertline){
long Fin;
long lsize;
long int lstart;		// point of insert line start
long int lend;			// point of insert line end
long int cur;			// current file pointer position

char dumbuf[INI_MAX_LINE+1];
char *	buffer;
size_t	result;
char	ch;

FILE*	fd;			// FD = original file; fd2 = file buffer for inserts
FILE*	fd2;
int		error;

    fd = fopen(inifile, "r");
    if (!fd)
        return -1;
    fd2 = fopen(inifilebuf, "w");
    if (!fd2)
        return -2;
    
	fseek (fd, 0L, SEEK_END);
	Fin = ftell(fd);					//  get the orig file size
	fseek(fd, 0L, SEEK_SET);			// go back to start of file
	
	// Search stream by parameter and return position of first character in file
	// lastart == 0(key found), -3 or -4 (key not found)
	lstart = ini_search_file(fd, section, name);	// search parameter in file
	if(lstart <0)
		return lstart;	// parameter not found: rc= -3 (no ']'found)
						// or -4 (key not found)

	// lstart delivers start pos of line; rd pointer is already at end of line
	lend = ftell(fd);					// and save this position
	lsize = Fin - lend;

	// copy start block to buffer file
	fseek(fd, 0L, SEEK_SET);
	while (1) {
      ch = fgetc(fd);
	  cur = ftell(fd);
	  
      if (ch == EOF || cur == lstart)
         break;
      else
         putc(ch, fd2);
	}
	fputs ("\n", fd2);
	fputs(insertline, fd2);				//insert value string

	fseek(fd, lend, SEEK_SET);			// go behind insert point
	
	// copy end block to buffer file
	while (lsize) {
      ch = fgetc(fd);
      if (ch == EOF)
         break;
      else{
         putc(ch, fd2);
		 lsize--;
	  }
	}
	fflush(fd2);
    fclose(fd);
	fclose(fd2);
	
	return 0;							// Done: string inserted
}

/*******************************************************************************
* Function: NewINI()
* Creation: 01.11.2017
* Author: R.Esser, 2019
* Creat new config.ini file of Beehive by default parameters
* !!! any format change of config ini requires change of this file as well !!!
*
* Input :
* inifile		name of default beehive config file
*
* Global:	-
*
* Output:
*	rc		NULL  no file coulf be created
*			ptr on global initialized configuration struct
*********************************************************************************
*/
configuration* newini(char* inifile){
#define	NEWCFGLINEMAX	100
#define CFGLINELEN		256
// allocate space for char pointers to strings
char * cfgline[NEWCFGLINEMAX];
int i = 0;
FILE *	bhcfg;

// allocate space for each cfgline string
    for(i = 0; i < NEWCFGLINEMAX; i++){
        cfgline[i] = (char*)malloc(CFGLINELEN * sizeof(char));
		*cfgline[i] = 0;	//reset all cfgline buffers
    }

	if(!(bhcfg = fopen(inifile, "w"))){
		//----- cfg file could not be created -----	
		return(NULL);
	}	// else new file created

	strcpy(cfgline[0],  "; BeeIoT Gateway&Server config file for first runtime initialization\n");
	strcpy(cfgline[1],  "; 01.02.2020\n");
	strcpy(cfgline[2],  "VERSION = '2.2'\n");
        strcpy(cfgline[3],  "VMAJOR	= 1		; Major Version of BIoTWAN Protocol\n");
	strcpy(cfgline[4],  "VMINOR	= 0		; Minor Version of BIoTWAN Protocol\n");
	strcpy(cfgline[5],  "\n");
	strcpy(cfgline[6],  "; BeeIoT Server configuration runtime parameters\n");
	strcpy(cfgline[7],  "; This file can be modified during runtime of beeiotgw !\n");
	strcpy(cfgline[8],  "; All Values are parsed cyclic during runtime right after the wait loop.\n");
	strcpy(cfgline[9], "\n");
	strcpy(cfgline[10], "[HWCONFIG]                 ; wPi configuration settings\n");
	strcpy(cfgline[11], "LORANUMCHN	 1		; only one LORA channel supported > single chn. gateway\n");
	strcpy(cfgline[12], "LORADEFCHN	 0		; Default LORA Channel for JOIN REQUEST\n");
	strcpy(cfgline[13], "; LoRa Port0 > Dragino LoRa Hat GPIO ports of SX1276 + GPS chip\n");
	strcpy(cfgline[14], "LORA0CS		= 6	; GPIO 25  	Pin 22\n");
	strcpy(cfgline[15], "LORA0MOSI	= 12	; GPIO 10 	MOSI Pin 19\n");
	strcpy(cfgline[16], "LORA0MISO	= 13	; GPIO 9  	MISO Pin 21\n");
	strcpy(cfgline[17], "LORA0SCK	= 14	; GPIO 11 	SCLK Pin 23 \n");
	strcpy(cfgline[18], "LORA0RST	= 0	; GPIO 0	Pin 11\n");
	strcpy(cfgline[19], "LORA0DIO0	= 7	; GPIO 4	Pin 7\n");
	strcpy(cfgline[20], "LORA0DIO1	= 4	; GPIO 23	Pin 16\n");
	strcpy(cfgline[21], "LORA0DIO2	= 5	; GPIO 24	Pin 18\n");
	strcpy(cfgline[22], "LORA0DIO3	= -1	; GPIO x	Pin x\n");
	strcpy(cfgline[23], "LORA0DIO4	= -1	; GPIO x	Pin x\n");
	strcpy(cfgline[24], "LORA0DIO5	= -1	; GPIO x	Pin x\n");
	strcpy(cfgline[25], "GPS0TXD	= 15	; GPIO 15 	TxD	of GPS module\n");
	strcpy(cfgline[26], "GPS0RXD	= 16	; GPIO 16 	RxD of GPS module\n");
	strcpy(cfgline[27], "\n");
	strcpy(cfgline[28], "; Component enabler\n");
	strcpy(cfgline[29], "HCLORA     = 1     ; =1 LoRa Port sending enabled\n");
	strcpy(cfgline[30], "HCLORAWAN  = 0	; =0 BIoTWAN protocol enabled, =1 LoRaWAN enable (not supp. yet)\n");
	strcpy(cfgline[31], "HCGPS      = 0	; =1 GPS module enabled\n");
	strcpy(cfgline[32], "HCLOCWEB   = 1	; =1 Activate local Webpage date preparation at BEEIOTWEB\n");
	strcpy(cfgline[33], "HCREMWEB   = 1	; =1 Activate remote Webpage date preparation at EXFTPURL > EXFTPPATH\n");
	strcpy(cfgline[34], "\n");
	strcpy(cfgline[35], "[BEEIOT]   ; Init of Main Loop\n");
	strcpy(cfgline[36], "BHLOOPWAIT = 600           ; loop wait time in Sec. (600 = 10Min.)\n");
	strcpy(cfgline[37], "BEEIOTHOME = /home/pi/share/biot  ; Home path for beeiotgw housekeeping data\n");
	strcpy(cfgline[38], "LOGFILE    = biot.txt      ; log file name (/w extension)\n");
	strcpy(cfgline[39], "CSVFILE    = biotlog       ; *.csv data log file name (/wo extension): results in beeiotlogYYYY.csv\n");
	strcpy(cfgline[40], "CSVDAYS    = biotdays      ; *.csv file of daily statistic summary (/wo extension)\n");
	strcpy(cfgline[41], "ACTIOFAIL  = 1             ; allowed action on IO Error: 0= no Action, 1= exit, 2=reboot\n");
	strcpy(cfgline[42], "VERBOSE	= 1             ; verbose levels +1=main flow + 2=OneWire + 4=hx711 + 8 ePaper\n");
        strcpy(cfgline[43], "                           ; 16=LANcfg + 32=SDCard + 64=ADS + 128=SPI Port\n");
        strcpy(cfgline[44], "                           ; 256=Lora-Radio + 512=Lora-BIoTWAN-Protocol\n");
	strcpy(cfgline[45], "[HX711]    ; Init of Weight scale ADC\n");
	strcpy(cfgline[46], "TARA       = 297570        ; Calibration for 0 kg\n");
	strcpy(cfgline[47], "TARASET    = 0             ; =1 TARA reset by last measured weight level -> at next loop\n");
	strcpy(cfgline[48], "REFKG      = 44000         ; weight scale reference value of 1kg\n");
	strcpy(cfgline[49], "TEMPCOMP   = 1.0           ; Temp. compensation factor per Grad\n");
	strcpy(cfgline[50], "NSAMPLES   = 1             ; Number of read loops for average calculation (2..100)\n"); 
	strcpy(cfgline[51], "\n");

	strcpy(cfgline[52], "[ONEWIRE]  ; Init of One-Wire devices\n");
	strcpy(cfgline[53], "TEMPCEXT     = 1.00        ; temperature compensation External sensor\n");
	strcpy(cfgline[54], "TEMPCINT     = 1.00        ; temperature compensation Internal sensor\n");
	strcpy(cfgline[55], "TEMPCHIVE    = 1.00        ; temperature compensation Hive1 sensor\n");
	strcpy(cfgline[56], "\n");

	strcpy(cfgline[57], "[WEBUI]\n");
	strcpy(cfgline[58], "AUTOUPDATE  = 0                 ; =1 automatic update of website\n");
	strcpy(cfgline[59], "BEEIOTWEB   = /var/www/html/beeiot   ; root path to webserver home of beeiot for log & data files\n");
	strcpy(cfgline[60], "BEEKEEPER   = 'UserName'        ; Full name of Owner/User/BeeKeeper\n");
	strcpy(cfgline[61], "LOCDAT1     = '-Garten-'        ; Location of BeeHive1\n");
	strcpy(cfgline[62], "LOCDAT2     = 'Strasse'         ; Street\n");
	strcpy(cfgline[63], "LOCPLZ      = 'PLZ'             ; ZIP code of location (also fro weather data from web)\n");
	strcpy(cfgline[64], "LOCDAT3     = 'Ort'             ; location name\n");
	strcpy(cfgline[65], "PICSMALL    = BeeIoT_Picture_compressed.jpg ; Pic of BeeHive (compressed) used as WebLogo\n");
	strcpy(cfgline[66], "PICLARGE    = BeeIoT_Picture.jpg ; Pix of Beehive full size\n");
	strcpy(cfgline[67], "WEBDEFFILE  = index.html        ; default Web index file to be updated\n");
	strcpy(cfgline[68], "NOTICEFILE  = beenote.txt       ; text file of service notices for logging\n");
	strcpy(cfgline[69], "ALARMON     = 0                 ; =1 Global 'ALARM enabled' for security/events\n");
	strcpy(cfgline[70], "ALARMWEIGHT = 0                 ; Alarm on Weight change > 50% in 5 seconds: thieve\n");
	strcpy(cfgline[71], "                                ; =0 disabled, 1..99% enabled, typical 50(%)\n");
	strcpy(cfgline[72], "ALARMSWARM  = 0                 ; Alarm on weight change > 10% in 10 minutes: swarm\n");
	strcpy(cfgline[73], "                                ; =0 disabled, 1..99% enabled, typical 10(%)\n");
	strcpy(cfgline[74], "ALARMBATT1  = 0                 ; =0 disabled; 1..100% enabled, typical 100(%)= 4.2V->Max\n");
	strcpy(cfgline[75], "ALARMBATT2  = 0                 ; =0 disabled; 1..100% enabled, typical 0(%)= 3.3V ->Off\n");
	strcpy(cfgline[76], "\n");

	strcpy(cfgline[77], "[EXPORT]\n");
	strcpy(cfgline[78], "EXFTPURL    = <ftp URL>         ; FTP site URL for upload of raw logger data from BEELOGWEB\n");
	strcpy(cfgline[79], "EXFTPPORT   = 21                ; Portnumber of URL (used as string)\n");
	strcpy(cfgline[80], "EXFTPPATH   = imkerei/beelog    ; relative FTP path to URL\n");
	strcpy(cfgline[81], "EXFTPPROXY  =                   ; If needed: FTP proxy server; set '' if no proxy needed\n");
	strcpy(cfgline[82], "EXFTPPROXYPORT =                ; used proxy port (used as string)\n");
	strcpy(cfgline[83], "EXFTPUSER   =                   ;no user name for FTP access (get pwd by dialogue or local .netrc file)\n");
	strcpy(cfgline[84], "BKUPPATH    = /home/pi/share/beeiot ; Backup file path (local or remote)\n");
	strcpy(cfgline[85], "BKUPFILE    = beeiot            ; name of config/log backup file at BKUPPATH -> beeiotlogYYYY.bak\n");
	
	for (i=0; i<NEWCFGLINEMAX; i++){	// write all cfglines to new cfg file
		fwrite(cfgline[i],sizeof(unsigned char), strlen(cfgline[i]), bhcfg) ;
		free(cfgline[i]);		// release string buffer of cfgline
	}
	
	fflush(bhcfg);
	fclose(bhcfg);
	return(&config);
} // endof newini()