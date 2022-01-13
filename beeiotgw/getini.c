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

		} else if (MATCH("HWCONFIG", "LORANUMCHN")) {
			pconfig->loranumchn = atoi(value);
		} else if (MATCH("HWCONFIG", "LORADEFCHN")) {
			pconfig->loradefchn = atoi(value);

		} else if (MATCH("HWCONFIG", "LORAxMISO")) {
			pconfig->loraxmosi = atoi(value);
		} else if (MATCH("HWCONFIG", "LORAxMOSI")) {
			pconfig->loraxmiso = atoi(value);
		} else if (MATCH("HWCONFIG", "LORAxSCK")) {
			pconfig->loraxclk = atoi(value);

		} else if (MATCH("HWCONFIG", "LORA0CS")) {
			pconfig->lora_cs0 = atoi(value);
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
		} else if (MATCH("HWCONFIG", "LORA0CHANNEL")) {
			pconfig->lora0channel = atoi(value);

		} else if (MATCH("HWCONFIG", "LORA1CS")) {
			pconfig->lora_cs1 = atoi(value);
		} else if (MATCH("HWCONFIG", "LORA1RST")) {
			pconfig->lora1rst = atoi(value);
		} else if (MATCH("HWCONFIG", "LORA1DIO0")) {
			pconfig->lora1dio0 = atoi(value);
		} else if (MATCH("HWCONFIG", "LORA1DIO1")) {
			pconfig->lora1dio1 = atoi(value);
		} else if (MATCH("HWCONFIG", "LORA1DIO2")) {
			pconfig->lora1dio2 = atoi(value);
		} else if (MATCH("HWCONFIG", "LORA1DIO3")) {
			pconfig->lora1dio3 = atoi(value);
		} else if (MATCH("HWCONFIG", "LORA1DIO4")) {
			pconfig->lora1dio4 = atoi(value);
		} else if (MATCH("HWCONFIG", "LORA1DIO5")) {
			pconfig->lora1dio5 = atoi(value);
		} else if (MATCH("HWCONFIG", "LORA1CHANNEL")) {
			pconfig->lora1channel = atoi(value);

		} else if (MATCH("HWCONFIG", "GPS0TXD")) {
			pconfig->gps0TX = atoi(value);
		} else if (MATCH("HWCONFIG", "GPS0RXD")) {
			pconfig->gps0RX = atoi(value);

		} else if (MATCH("HWCONFIG", "ND1_GWID")) {
			pconfig->nd1_gwid = atoi(value);
		} else if (MATCH("HWCONFIG", "ND1_MID")) {
			pconfig->nd1_mid = atoi(value);
		} else if (MATCH("HWCONFIG", "ND1_APPEUI")) {
			pconfig->nd1_appeui = atoi(value);
		} else if (MATCH("HWCONFIG", "ND1_DEVEUI1")) {
			pconfig->nd1_deveuiup = strtoul(value, NULL, 16);
		} else if (MATCH("HWCONFIG", "ND1_DEVEUI2")) {
			pconfig->nd1_deveuilo = strtoul(value, NULL, 16);
		} else if (MATCH("HWCONFIG", "ND1_FREPORT")) {
			pconfig->nd1_freport = atoi(value);
		} else if (MATCH("HWCONFIG", "ND1_WCALIB")) {
			pconfig->nd1_wcalib = atoi(value);
		} else if (MATCH("HWCONFIG", "ND1_HWCONF")) {
			pconfig->nd1_hwconfig = atoi(value);

		} else if (MATCH("HWCONFIG", "ND2_GWID")) {
			pconfig->nd2_gwid = atoi(value);
		} else if (MATCH("HWCONFIG", "ND2_MID")) {
			pconfig->nd2_mid = atoi(value);
		} else if (MATCH("HWCONFIG", "ND2_APPEUI")) {
			pconfig->nd2_appeui = atoi(value);
		} else if (MATCH("HWCONFIG", "ND2_DEVEUI1")) {
			pconfig->nd2_deveuiup = strtoul(value, NULL, 16);
		} else if (MATCH("HWCONFIG", "ND2_DEVEUI2")) {
			pconfig->nd2_deveuilo = strtoul(value, NULL, 16);
		} else if (MATCH("HWCONFIG", "ND2_FREPORT")) {
			pconfig->nd2_freport = atoi(value);
		} else if (MATCH("HWCONFIG", "ND2_WCALIB")) {
			pconfig->nd2_wcalib = atoi(value);
		} else if (MATCH("HWCONFIG", "ND2_HWCONF")) {
			pconfig->nd2_hwconfig = atoi(value);

		} else if (MATCH("HWCONFIG", "ND3_GWID")) {
			pconfig->nd3_gwid = atoi(value);
		} else if (MATCH("HWCONFIG", "ND3_MID")) {
			pconfig->nd3_mid = atoi(value);
		} else if (MATCH("HWCONFIG", "ND3_APPEUI")) {
			pconfig->nd3_appeui = atoi(value);
		} else if (MATCH("HWCONFIG", "ND3_DEVEUI1")) {
			pconfig->nd3_deveuiup = strtoul(value, NULL, 16);
		} else if (MATCH("HWCONFIG", "ND3_DEVEUI2")) {
			pconfig->nd3_deveuilo = strtoul(value, NULL, 16);
		} else if (MATCH("HWCONFIG", "ND3_FREPORT")) {
			pconfig->nd3_freport = atoi(value);
		} else if (MATCH("HWCONFIG", "ND3_WCALIB")) {
			pconfig->nd3_wcalib = atoi(value);
		} else if (MATCH("HWCONFIG", "ND3_HWCONF")) {
			pconfig->nd3_hwconfig = atoi(value);

		} else if (MATCH("HWCONFIG", "ND4_GWID")) {
			pconfig->nd4_gwid = atoi(value);
		} else if (MATCH("HWCONFIG", "ND4_MID")) {
			pconfig->nd4_mid = atoi(value);
		} else if (MATCH("HWCONFIG", "ND4_APPEUI")) {
			pconfig->nd4_appeui = atoi(value);
		} else if (MATCH("HWCONFIG", "ND4_DEVEUI1")) {
			pconfig->nd4_deveuiup = strtoul(value, NULL, 16);
		} else if (MATCH("HWCONFIG", "ND4_DEVEUI2")) {
			pconfig->nd4_deveuilo = strtoul(value, NULL, 16);
		} else if (MATCH("HWCONFIG", "ND4_FREPORT")) {
			pconfig->nd4_freport = atoi(value);
		} else if (MATCH("HWCONFIG", "ND4_WCALIB")) {
			pconfig->nd4_wcalib = atoi(value);
		} else if (MATCH("HWCONFIG", "ND4_HWCONF")) {
			pconfig->nd4_hwconfig = atoi(value);
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
		} else if (MATCH("BEEIOT", "CMDFILE")) {
			strcpy(pconfig->biot_CMDFILE,value);
		} else if (MATCH("BEEIOT", "RESULTFILE")) {
			strcpy(pconfig->biot_RESULTFILE,value);
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
* Create a new config.ini file of Beehive by default parameters
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
	strcpy(cfgline[1],  "; 28.02.2021\n");
	strcpy(cfgline[2],  "VERSION = '2.5'\n");
    strcpy(cfgline[3],  "VMAJOR	= 1		; Major Version of BIoTWAN Protocol\n");
	strcpy(cfgline[4],  "VMINOR	= 4		; Minor Version of BIoTWAN Protocol\n");
	strcpy(cfgline[5],  "\n");
	strcpy(cfgline[6],  "; BeeIoT Server configuration runtime parameters\n");
	strcpy(cfgline[7],  "; This file can be modified during runtime of beeiotgw !\n");
	strcpy(cfgline[8],  "; All Values are parsed cyclic during runtime right after the wait loop.\n");
	strcpy(cfgline[9],  "\n");
	strcpy(cfgline[10], "[HWCONFIG]         ; wPi configuration settings\n");
	strcpy(cfgline[11], "LORANUMCHN	 2		; only one LORA channel supported > single chn. gateway\n");
	strcpy(cfgline[12], "LORADEFCHN	 0		; Default LORA Channel for JOIN REQUEST\n\n");

	strcpy(cfgline[13], "LORAxMISO	= 13	; GPIO 9 	MISO Pin 21\n");
	strcpy(cfgline[14], "LORAxMOSI	= 12	; GPIO 10  	MOSI Pin 19\n");
	strcpy(cfgline[15], "LORAxSCK	= 14	; GPIO 11 	SCLK Pin 23 \n\n");

	strcpy(cfgline[16], "; LoRa Port0 >wPi#:       BCM:	J40:	=> Radio Modem 0\n");
	strcpy(cfgline[17], "LORA0CS	= 6	; GPIO 25  	Pin 22\n");
	strcpy(cfgline[18], "LORA0RST	= 0	; GPIO 0	Pin 11\n");
	strcpy(cfgline[19], "LORA0DIO0	= 7	; GPIO 4	Pin 7\n");
	strcpy(cfgline[20], "LORA0DIO1	= 4	; GPIO 23	Pin 16\n");
	strcpy(cfgline[21], "LORA0DIO2	= 5	; GPIO 24	Pin 18\n");
	strcpy(cfgline[22], "LORA0DIO3	= -1	; GPIO x	Pin x\n");
	strcpy(cfgline[23], "LORA0DIO4	= -1	; GPIO x	Pin x\n");
	strcpy(cfgline[24], "LORA0DIO5	= -1	; GPIO x	Pin x\n");
	strcpy(cfgline[25], "LORA0CHANNEL= 0	; Channel cfg. set (see BeeIoTWAN.h)\n\n");

	strcpy(cfgline[26], "; LoRa Port1 >wPi#:	      BCM:	J40:	=> Radio Modem 1\n");
	strcpy(cfgline[27], "LORA1CS		= 21	; GPIO 5  	Pin 29	NSS1\n");
	strcpy(cfgline[28], "LORA1RST	= 22	; GPIO 6	Pin 31	Reset1\n");
	strcpy(cfgline[29], "LORA1DIO0	= 23	; GPIO 13	Pin 33	DIO0-1\n");
	strcpy(cfgline[30], "LORA1DIO1	= -1	; GPIO x	Pin x	DIO1-1 n.c.\n");
	strcpy(cfgline[31], "LORA1DIO2	= -1	; GPIO x	Pin x	DIO2-1 n.c.\n");
	strcpy(cfgline[32], "LORA1DIO3	= -1	; GPIO x	Pin x	DIO3-1 n.c.\n");
	strcpy(cfgline[33], "LORA1DIO4	= -1	; GPIO x	Pin x	DIO4-1 n.c.\n");
	strcpy(cfgline[34], "LORA1DIO5	= -1	; GPIO x	Pin x	DIO5-1 n.c.\n");
	strcpy(cfgline[35], "LORA1CHANNEL= 1		; Channel cfg. set (see BeeIoTWAN.h)\n\n");

	strcpy(cfgline[36], "GPS0TXD	= 15	; GPIO 15 	TxD	of GPS module\n");
	strcpy(cfgline[37], "GPS0RXD	= 16	; GPIO 16 	RxD of GPS module\n\n");

	strcpy(cfgline[38], "; LoRa Client Registration:\n");
	strcpy(cfgline[39], "; Node 1: BeeIoT ESP32-WROOM32:	MAC: 24:6F:28:D5:8A:DC	default: BeeHive Weightcell #1\n");
	strcpy(cfgline[40], "ND1_GWID	= 1		; GW ID relative to GWIDx (1..LORANUMCHN)\n");
	strcpy(cfgline[41], "ND1_MID		= 0		; Assign physical Radio Modem to virt. GW\n");
	strcpy(cfgline[42], "ND1_APPEUI  = 1		; 1= BIOT, 2= TURTLE, 3= GH\n");
	strcpy(cfgline[43], "ND1_DEVEUI1 = DC8AD5FF	; Unique DEVEUI of Node Upper Long\n");
	strcpy(cfgline[44], "ND1_DEVEUI2 = FE286F24	; Unique DEVEUI of Node Lower Long\n");
	strcpy(cfgline[45], "ND1_FREPORT = 10	; Report Frequency in Minutes\n");
	strcpy(cfgline[46], "ND1_WCALIB	= 0		; Weight Cell calibration (absolute +/-)\n\n");

	strcpy(cfgline[47], "; Node 2: ESP32-WROVERB: MAC 24:6F:28:F0:0D:AC		beacon test Module 1\n");
	strcpy(cfgline[48], "ND2_GWID	= 1		; GW ID relative to GWIDx (1..LORANUMCHN)\n");
	strcpy(cfgline[49], "ND2_MID		= 0		; Assign physical Radio Modem to virt. GW\n");
	strcpy(cfgline[50], "ND2_APPEUI  = 1		; 1= BIOT, 2= TURTLE, 3= GH\n");
	strcpy(cfgline[51], "ND2_DEVEUI1 = AC0DF0FF	; Unique DEVEUI of Node Upper Long\n");
	strcpy(cfgline[52], "ND2_DEVEUI2 = FE286F24	; Unique DEVEUI of Node Lower Long\n");
	strcpy(cfgline[53], "ND2_FREPORT = 1	    ; Report Frequency in Minutes\n");
	strcpy(cfgline[54], "ND2_WCALIB	= 0		; Weight Cell calibration (absolute +/-)\n\n");

	strcpy(cfgline[55], "; Node 3: BeeIoT ESP32-WROOM32:	MAC: 94:FE:8A:B5:AA:8C	Beacon test Module 2\n");
	strcpy(cfgline[56], "ND3_GWID	= 1		; GW ID relative to GWIDx (1..LORANUMCHN)\n");
	strcpy(cfgline[57], "ND3_MID		= 0		; Assign physical Radio Modem to virt. GW\n");
	strcpy(cfgline[58], "ND3_APPEUI  = 1		; 1= BIOT, 2= TURTLE, 3= GH\n");
	strcpy(cfgline[59], "ND3_DEVEUI1 = 94FE8AFF	; Unique DEVEUI of Node Upper Long\n");
	strcpy(cfgline[60], "ND3_DEVEUI2 = FEB5AA8C	; Unique DEVEUI of Node Lower Long\n");
	strcpy(cfgline[61], "ND3_FREPORT = 1	    ; Report Frequency in Minutes\n");
	strcpy(cfgline[62], "ND3_WCALIB	= 0		; Weight Cell calibration (absolute +/-)\n\n");

	strcpy(cfgline[63], "; Node 4: BeeIoT ESP32-WROOM32:	MAC: 2C:2B:16:28:6F:24 	Beehive Weight cell test Module 3\n");
	strcpy(cfgline[64], "ND4_GWID	= 2		; GW ID relative to GWIDx (1..LORANUMCHN)\n");
	strcpy(cfgline[65], "ND4_MID		= 1		; Assign physical Radio Modem to virt. GW\n");
	strcpy(cfgline[66], "ND4_APPEUI  = 1		; 1= BIOT, 2= TURTLE, 3= GH\n");
	strcpy(cfgline[67], "ND4_DEVEUI1 = 2C2B16FF	; Unique DEVEUI of Node Upper Long\n");
	strcpy(cfgline[68], "ND4_DEVEUI2 = FE286F24	; Unique DEVEUI of Node Lower Long\n");
	strcpy(cfgline[69], "ND4_FREPORT = 10	; Report Frequency in Minutes\n");
	strcpy(cfgline[70], "ND4_WCALIB	= 0		; Weight Cell calibration (absolute +/-)\n\n");

	strcpy(cfgline[71], "; Component enabler\n");
	strcpy(cfgline[72], "HCLORA       = 1   ; =1 LoRa Port sending enabled\n");
	strcpy(cfgline[73], "HCLORAWAN    = 0	; =0 BIoTWAN protocol enabled, =1 LoRaWAN enable (not supp. yet)\n");
	strcpy(cfgline[74], "HCWIFICLIENT = 1	; =1 Client (!) onboard Wifi to be used\n");
	strcpy(cfgline[75], "HCGPS        = 0	; =1 GPS module enabled\n");
	strcpy(cfgline[76], "HCLOCWEB     = 1	; =1 Activate local Webpage date preparation at BEEIOTWEB\n");
	strcpy(cfgline[77], "HCREMWEB     = 1	; =1 Activate remote Webpage date preparation at EXFTPURL > EXFTPPATH\n\n");
	strcpy(cfgline[78], "[BEEIOT]   ; Init of Main Loop\n");
	strcpy(cfgline[79], "BHLOOPWAIT = 10            ; loop wait time in Minutes\n");
	strcpy(cfgline[80], "BEEIOTHOME = /home/pi/biot ; Home path for beeiotgw housekeeping data\n");
	strcpy(cfgline[81], "LOGFILE    = beeiot.txt    ; log file name (/w extension)\n");
	strcpy(cfgline[82], "CSVFILE    = beeiotlog     ; *.csv data log file name (/wo extension): results in beeiotlogYYYY.csv\n");
	strcpy(cfgline[83], "CSVDAYS    = beeiotdays    ; *.csv file of daily statistic summary (/wo extension)\n");
	strcpy(cfgline[84], "ACTIOFAIL  = 1             ; allowed action on IO Error: 0= no Action, 1= exit, 2=reboot\n");
	strcpy(cfgline[85], "VERBOSE	= 1             ; verbose levels +1=main flow + 2=OneWire + 4=hx711 + 8 ePaper\n");
    strcpy(cfgline[86], "                           ; 16=LANcfg + 32=SDCard + 64=ADS + 128=SPI Port\n");
    strcpy(cfgline[87], "                           ; 256=Lora-Radio + 512=Lora-BIoTWAN-Protocol\n\n");
	strcpy(cfgline[81], "CMDFILE    = beecmd.txt    ; cmd file name (/w extension)\n");
	strcpy(cfgline[81], "RESULTFILE = beeresult.txt ; result file name (/w extension)\n");

	strcpy(cfgline[88], "[WEBUI]\n");
	strcpy(cfgline[89], "AUTOUPDATE  = 0                 ; =1 automatic update of website\n");
	strcpy(cfgline[90], "BEEIOTWEB   = /var/www/html/beeiot   ; root path to webserver home of beeiot for log & data files\n");
	strcpy(cfgline[91], "BEEKEEPER   = 'UserName'        ; Full name of Owner/User/BeeKeeper\n");
	strcpy(cfgline[92], "LOCDAT1     = '-Garten-'        ; Location of BeeHive1\n");
	strcpy(cfgline[93], "LOCDAT2     = 'Strasse'         ; Street\n");
	strcpy(cfgline[94], "LOCPLZ      = 'PLZ'             ; ZIP code of location (also fro weather data from web)\n");
	strcpy(cfgline[95], "LOCDAT3     = 'Ort'             ; location name\n");
	strcpy(cfgline[96], "PICSMALL    = BeeIoT_Picture_compressed.jpg ; Pic of BeeHive (compressed) used as WebLogo\n");
	strcpy(cfgline[97], "PICLARGE    = BeeIoT_Picture.jpg ; Pix of Beehive full size\n");
	strcpy(cfgline[98], "WEBDEFFILE  = index.html        ; default Web index file to be updated\n");
	strcpy(cfgline[99], "NOTICEFILE  = beenote.txt       ; text file of service notices for logging\n");
	strcpy(cfgline[100], "ALARMON     = 0                 ; =1 Global 'ALARM enabled' for security/events\n");
	strcpy(cfgline[101], "ALARMWEIGHT = 0                 ; Alarm on Weight change > 50% in 5 seconds: thieve\n");
	strcpy(cfgline[102], "                                ; =0 disabled, 1..99% enabled, typical 50(%)\n");
	strcpy(cfgline[103], "ALARMSWARM  = 0                 ; Alarm on weight change > 10% in 10 minutes: swarm\n");
	strcpy(cfgline[104], "                                ; =0 disabled, 1..99% enabled, typical 10(%)\n");
	strcpy(cfgline[105], "ALARMBATT1  = 0                 ; =0 disabled; 1..100% enabled, typical 100(%)= 4.2V->Max\n");
	strcpy(cfgline[106], "ALARMBATT2  = 0                 ; =0 disabled; 1..100% enabled, typical 0(%)= 3.3V ->Off\n\n");

	strcpy(cfgline[107], "[EXPORT]\n");
	strcpy(cfgline[108], "EXFTPURL    = <ftp URL>         ; FTP site URL for upload of raw logger data from BEELOGWEB\n");
	strcpy(cfgline[109], "EXFTPPORT   = 21                ; Portnumber of URL (used as string)\n");
	strcpy(cfgline[110], "EXFTPPATH   = imkerei/beelog    ; relative FTP path to URL\n");
	strcpy(cfgline[111], "EXFTPPROXY  =                   ; If needed: FTP proxy server; set '' if no proxy needed\n");
	strcpy(cfgline[112], "EXFTPPROXYPORT =                ; used proxy port (used as string)\n");
	strcpy(cfgline[113], "EXFTPUSER   =                   ;no user name for FTP access (get pwd by dialogue or local .netrc file)\n");
	strcpy(cfgline[114], "BKUPPATH    = /home/pi/beeiot   ; Backup file path (local or remote)\n");
	strcpy(cfgline[115], "BKUPFILE    = beeiot.bak        ; name of config/log backup file at BKUPPATH -> beeiotlogYYYY.bak\n");
	
	for (i=0; i<NEWCFGLINEMAX; i++){	// write all cfglines to new cfg file
		fwrite(cfgline[i],sizeof(unsigned char), strlen(cfgline[i]), bhcfg) ;
		free(cfgline[i]);		// release string buffer of cfgline
	}
	
	fflush(bhcfg);
	fclose(bhcfg);
	return(&config);
} // endof newini()