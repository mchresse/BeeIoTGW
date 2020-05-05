/*******************************************************************************
* The "BeeIoTWAN_GW" Module is distributed under the New BSD license:
*
* Copyright (c) 2020, Randolph Esser
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
* BIoTApp.cpp is released under the New BSD license. Go to the project
* home page for more info: https://github.com/beeiot_GW
*     https://github.com/mchresse/BeeIoT_GW/license
*/
//*******************************************************************
// BeeIoTWan_GW project 
// from Project https://github.com/mchresse/BeeIoT_GW
//
// Description:
// BeeIoT-WAN - Gateway and NwServer based on Raspi
//
// Created on 1. February 2020
//----------------------------------------------------------
// Copyright (c) 2019-present, Randolph Esser
// All rights reserved.
// This file is distributed under the BSD-3-Clause License
// The complete license agreement can be obtained at: 
// For used 3rd party open source see also Readme_OpenSource.txt
//*******************************************************************

#include <iostream>
#include <cstdint>
#include <cstring>
#include <sys/time.h>

#include <sys/ioctl.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <linux/sockios.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>

#include "BeeIoTWan.h"
#include "beeiot.h"
#include "gwqueue.h"
#include "beelora.h"
#include "regslora.h"
#include "radio.h"
#include "NwSrv.h"
#include "JoinSrv.h"
#include "BIoTApp.h"

//******************************************************************************
// BIoTWAN global variables
unsigned int	lflags;			// BeeIoT log flag field

// Central Database of all measured values and runtime parameters
dataset			bhdb;			// central beeIoT data DB

// The one and only global init parameter-set buffer parsed from config.ini file
configuration * cfgini;			// ptr. to struct with initial parameters


#define MAXMSGLEN	1024		// length of univ. message buffer
char	csv_notice[MAXMSGLEN];	// free notice field for *.csv file
	
//******************************************************************************
// BIoTWAN local variables

// list of bound services / Gateway
gwbind_t gwtab;	// ptr. forwarded to all service constructors


// to retrieve local LAN Port MAC address -> also used by BIoTApp.cpp
struct sockaddr_in si_other;
int s, slen = sizeof(si_other);
struct ifreq ifr;

static struct timeval now;		// current tstamp used each time a time check is done
static char	  TimeString[128];	// contains formatted Timestamp string of each loop(in main())


//******************************************************************************
//  Global Function prototypes
// API to NwSrv:
extern int  NwNodeScan (modemcfg_t * gwset, int nmodem, int defchannel);
// API to BeeLog:
int beelog(char * comment);

//******************************************************************************
//  Local Function prototypes
int  initall();
int  setmodemcfg(modemcfg_t * modem);
void BIoT_die(int rc);

/******************************************************************************
 * MAIN()
 ******************************************************************************/
int main () {
	char	msg[MAXMSGLEN];		// universals string buffer for log line creation	
	char *	notice;				// notice buffer for logfile and csv comment fields
	int		rc;

	
	setbuf(stdout, 0);		// disable buffering on stdout
	lflags = LOGBH;   // Define Log level (search for Log values in beeiot.h)
//  lflags = LOGBH + LOGOW + LOGHX + LOGLAN + LOGEPD + LOGSD + LOGADS + LOGSPI + LOGLORAR + LOGLORAW;

	// get current timestamp
	gettimeofday(&now, 0);
	strftime(TimeString, 80, "%d-%m-%y %H:%M:%S", localtime(&now.tv_sec));

	 // read of config.ini runtime parameters and setup all global structs
	rc = initall();
	if(rc != 0){	// setup BHDB + gwset[]
		BHLOG(LOGBH) printf("Main: InitAll failed with RC: %i\n", rc); 
		exit(-1);	// Initialization failed
	}
	BHLOG(LOGBH) printf("  %s: Setup starts with BIoTWAN v%d.%d\n", 
		TimeString,  (int)BIoT_VMAJOR, (int)BIoT_VMINOR);	


	sprintf(msg, "BIoT started", TimeString);
	beelog (msg);	// remember used offset 

	// Get Local LAN Port settings
    if ( (s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
        printf("  No LAN socket detected !!!\n");	// if no LAN Port -> do nothing
		ifr.ifr_hwaddr.sa_data[0] = 0;
		// -> no LAN Port => BIoTWan communication only (no upload)
    }else{
		// -> LAN Port => BIoTWan communication only (no upload)
		memset((char *) &si_other, 0, sizeof(si_other));
		si_other.sin_family = AF_INET;
		// to be activated if TTN communication; not supp. by now.
		//		si_other.sin_port = htons(TTNPORT);

		// get MAC address for NwsEUI generation
		ifr.ifr_addr.sa_family = AF_INET;
		strncpy(ifr.ifr_name, "eth0", IFNAMSIZ-1);  // can we rely on eth0 only?
		ioctl(s, SIOCGIFHWADDR, &ifr);

		/* display Local GW MAC */
		 BHLOG(LOGLAN)printf("  Detected LAN Port MAC: %.2x:%.2x:%.2x:ff:ff:%.2x:%.2x:%.2x\n",
			   (unsigned char)ifr.ifr_hwaddr.sa_data[0],
			   (unsigned char)ifr.ifr_hwaddr.sa_data[1],
			   (unsigned char)ifr.ifr_hwaddr.sa_data[2],
			   (unsigned char)ifr.ifr_hwaddr.sa_data[3],
			   (unsigned char)ifr.ifr_hwaddr.sa_data[4],
			   (unsigned char)ifr.ifr_hwaddr.sa_data[5]);
		 
		 strncpy(bhdb.macaddr, ifr.ifr_hwaddr.sa_data, LENMACADDR);
	}

	// Run NwNodeScan(): BIoT WAN Node scan routine forever: 
	// Now Scan for incoming packages via radio_irq_handler()
	gwtab.nws->NwNodeScan();

	// this point will be never reached
	// clean up NwS & gwset[] objects
	BIoT_die(0);
} // end of main ()


/*******************************************************************************
* Function: InitAll()
* Creation: 29.02.2020
* 
* Initialize all program related infrastructure and runtime settings:
* - setup WiringPi Lib
* - get config.ini start runtime parameters
* - init all IO modules
*   
* Input :	none
* Global:	
*	cfgini		struct of runtime parameters	-> get runtime params
*	bhdb		virt. database of statistic data -> set to "no data"
*	alarmflag	field of alarmflags dep. on type.-> set to 0
*   csv_notice	field for service notices for csv-file logging
*
* Output:
*	0		Init successful
*	-1		something essential went wrong -> break main
*********************************************************************************
*/
int initall(){
int		i,rc=0;
char	sbuf[MAXMSGLEN];

	
	// retrieve initial config runtime settings
	cfgini = getini((char *)CONFIGINI);
	if( cfgini == NULL){
		printf("    BeeIoT-Init: INI FIle not found at: %s\n", CONFIGINI);
		exit(EXIT_FAILURE);
	}
	lflags		= (unsigned int) cfgini->biot_verbose;	// now we have the custom verbose mode

	printf("============== BeeIoT WAN Gateway v%i.%i (%s: v%i.%i) ===============\n\n",
			BIoT_VMAJOR, BIoT_VMINOR, (char *) CONFIGINI, cfgini->vmajor, cfgini->vminor);
	snprintf(sbuf, MAXMSGLEN, "##### BeeIoT WAN Server v%i.%i - started (Cfg. v%s) #####", 
			(int)cfgini->vmajor, (int)cfgini->vminor, cfgini->version);  
	beelog(sbuf);	// log init of program log file


	//  setup Wiring-PI Lib
	if(wiringPiSetup() < 0) {    // using wPi pin mapping
		BHLOG(LOGBH) printf("BeeIoT-Init: wiringPi-Init for wPi# Error\n");
		beelog((char *) " - Exit: Wiring-Pi Lib GPIO init failed\n");
		if(cfgini->biot_actionIOfailure != AOF_NOOP){
			exit(EXIT_FAILURE);
		}
	}	

//	reset datastor, alarmflag field and weather forecast field
//		initbhdb(&bhdb);
	bhdb.loopid		= -1;			// get sequential index of sample data set
	bhdb.packid		= -1;			// get sequential index of payload packages
	bhdb.macaddr[0]	= 0;
	bhdb.ipaddr[0]	= 0;			// no IP yet
	bhdb.BoardID	= 0;			// no board ID yet
    bhdb.NodeID     = 0;
	bhdb.date[0]	= 0;
	bhdb.time[0]	= 0;
	bhdb.formattedDate[0]=0;
	bhdb.packid		= -1;
	

// Initialize Gateway Services by layer binding order:
//	0. get GPIO settings from config.ini
//	1.	NwSrv()		-> Radio() (for all modems)
//	2.  MsgQueue()	-> MsgBuffer()
//	3.	JoinSrv()
//	4.	AppSrv()
//  5. Bind all Service instances together: using gwset[]
	
	sprintf(csv_notice, "started");	// prepare "Start" notice of new value series	

	// 0. initialize GateWay config sets by cfgini
	if (cfgini->loranumchn > MAXGW) {	// limit user channel ID to MAX supported in code
		cfgini->loranumchn = MAXGW;
	}
	gwtab.gwset = new modemcfg_t[cfgini->loranumchn];	// get space for # of supported Gateway cfg sets
	gwtab.nmodemsrv = 0;	// by now no active modems

	// Initialize GW Config table for max. # of GW (=modem) slots (virtual)
	for(int i = 0; i < cfgini->loranumchn; i++){
		gwtab.modem[i] = (Radio*)NULL;		// [n+1] set to NULL

		gwtab.gwset[i].modemid	= i;		// for i==0: single Modem mode: but multi GWids assigned
		gwtab.gwset[i].gwid		= GWIDx - i;// set default GWID (for multi mode only)
		setmodemcfg(&gwtab.gwset[i]);		// set iopins[] GPIO values & chncfgid
	} // end of gwset[] init loop	

	// Statistic counter: to be initialized/updated once per MsgQueue 
	// at NwSrv runtime during RX package handling (not TX)
    gwtab.cp_nb_rx_rcv	= 0;	// # received packages / modem
    gwtab.cp_nb_rx_ok	= 0;	// # of correct received packages
    gwtab.cp_up_pkt_fwd = 0;	// # of sent status packages to REST/WEb service
    gwtab.cp_nb_rx_bad	= 0;	// # of invalid RX packages
    gwtab.cp_nb_rx_crc	= 0;	// # of RX packages /w CRC error
	
	// 1. Create Network Service: NwSrv  (one for all modem instances)
	try{ 
		 // Get Gateway/Modem instances: NwSrv() -> MsgQueue() -> MsgBuffer()
		 // gwtab.modem[] will be set for all activated modems implicit by NwSrv()
		gwtab.nws = new NwSrv(gwtab, cfgini->loranumchn);
	} 
	catch (int & excode){
		switch(excode){
			case EX_NWSRV_INIT1:	// wrong NwSrv parameters
				BIoT_die(-2);
			case EX_NWSRV_INIT2:	// WiringPiSPISetup() failed
				BIoT_die(-3);			
			case EX_NWSRV_INIT3:	// no active modem left
				BIoT_die(-4);
			default:{				// other unknown exception
				std::exception();	// forward to system
				BIoT_die(-1);
			}
		}
	}
	catch (exception &e){
		cout << e.what();
		BIoT_die(-1);
	}// end of try()/catch() list
	
	gwtab.nmodemsrv = gwtab.nws->NwSrvModems();	// get # of active modems by NwSrv
	if(cfgini->loradefchn > gwtab.nmodemsrv) {   // User def.JOIN modem ID > out of range?
		cfgini->loradefchn = 0;	// overrule it by internal default: 0
	}
	gwtab.joindef = cfgini->loradefchn;	// get user default JOIN modem id
	

	// 2. Create MSG Queue: MsgQueue (one for all modem instances)
	try {
		gwtab.gwq = new MsgQueue();
	} catch (int & excode){
		switch(excode){
			case EX_MSGQU_INIT:{
				printf("  Main: MsgQueue Exception (0x%04X) received\n", (unsigned int)excode); 
				if(gwtab.nmodemsrv==0){
					printf("  Main: Could not create MsgQueue for first Modem -> STOP BIoT Service\n");
					BIoT_die(-5);	// no MsgQueue -> give up
				}
				break;
			}
			default:{				// other unknown exception
				std::exception();	// forward to system
				BIoT_die(-1);
			}
		}
	}// end of try()/catch()

	// 3. Create Join Service: JoinSrv (one for all modem instances)
	try{ 
		gwtab.jsrv =	new JoinSrv(gwtab, gwtab.nmodemsrv);
	} catch (int & excode){
		switch(excode){
			case EX_JSRV_INIT:{		// wrong JoinSrv() parameters
				BIoT_die(-6);
			}
			default:{				// other unknown exception
				std::exception();	// forward to system
				BIoT_die(-1);
			}
		}
	}// end of try()/catch()

	// 4. Create App Services: BIoTApp (one for all modem instances)
	try{ 
		gwtab.apps = new AppSrv(gwtab);
	} catch (int & excode){
		switch(excode){
			case EX_APPS_INIT:{		// BIoTApp() init failed
				BIoT_die(-7);
			}
			default:{				// other unknown exception
				std::exception();	// forward to system
				BIoT_die(-1);
			}
		}
	}// end of try()/catch()

	// Now all Services are bound together in gwtab
	// - Assign MsgQueue to LoRa Modem
	for(int i=0; i<gwtab.nmodemsrv; i++){
		gwtab.modem[i]->assign_gwqueue(gwtab.gwq);// assign Queue ref. to Modem	
		BHLOG(LOGQUE) gwtab.gwq->PrintStatus();	  // show initial Queue status		
	}
	return(0);
} // end of InitAll()

// Central Exit routine for BIoT
// INPUT:
//  rc		Exit code
//  maxmid	# of activated modems
void BIoT_die(int rc){
	delete gwtab.jsrv;
	delete gwtab.nws;
//	delete[] gwtab.modem;	// all modem are deleted implicit by ~NwSrv()
	delete gwtab.gwq;
	delete[] gwtab.gwset;
	exit(rc);
}



//************************************************************
// SetModemCfg()
// Init modem HW IO cfg settings based on User cfgini.ini file
// Global: struct cfgini
// Return:
//	=0: configuration complete
//	-1	no cfgini data struct
//	-2  wrong ModemID defined by User config.ini file
//	-3	wrong ModemID in structure must be between 0..cfgini->loranumchn-1
int setmodemcfg(modemcfg_t * modem){
	if (!cfgini)	
		return(-1);

	switch(modem->modemid){
		case 0:
			modem->iopins.sxcs		= cfgini->lora_cs0;
			modem->iopins.sxrst		= cfgini->lora0rst;
			modem->iopins.sxdio0	= cfgini->lora0dio0;
			modem->iopins.sxdio1	= cfgini->lora0dio1;
			modem->iopins.sxdio2	= cfgini->lora0dio2;
			modem->iopins.sxdio3	= cfgini->lora0dio3;
			modem->iopins.sxdio4	= cfgini->lora0dio4;
			modem->iopins.sxdio5	= cfgini->lora0dio5;
			modem->chncfgid			= cfgini->lora0channel;
			break;
		case 1:
			modem->iopins.sxcs		= cfgini->lora_cs1;
			modem->iopins.sxrst		= cfgini->lora1rst;
			modem->iopins.sxdio0	= cfgini->lora1dio0;
			modem->iopins.sxdio1	= cfgini->lora1dio1;
			modem->iopins.sxdio2	= cfgini->lora1dio2;
			modem->iopins.sxdio3	= cfgini->lora1dio3;
			modem->iopins.sxdio4	= cfgini->lora1dio4;
			modem->iopins.sxdio5	= cfgini->lora1dio5;
			modem->chncfgid			= cfgini->lora1channel;
			break;
		default: // don't know what to do  
			return(-3);
	}
	// define Mode & Pwrlevel per IO port
	pinMode(modem->iopins.sxcs,  OUTPUT);
	pinMode(modem->iopins.sxrst, OUTPUT);
	pinMode(modem->iopins.sxdio0,INPUT);
	pinMode(modem->iopins.sxdio1,INPUT);
	pinMode(modem->iopins.sxdio2,INPUT);
	digitalWrite(modem->iopins.sxcs,  HIGH);	// active low
	digitalWrite(modem->iopins.sxrst, HIGH);	// active low

	return(0);
}




//*************************************************************************
// PrintHex()
// Print bin field: pbin[0] <-> pbin[len-1] by given direction in hexadez.
// dump format: '0x-xx-xx-xx-xx-xx....'
// Print starts where cursor is and no EOL is used.
// INPUT
//    pbin    byte ptr on binary field[0]
//    bytelen number of 2 digit bytes to be printed
// (0)dir     =0 -> forward  [0...len-1],   =1 -> backward [len-1...0]
// (1)format  =1: bytewise  =2: 2bytes(short)  =4bytes(word) =8bytes(long)...
//*************************************************************************
void Printhex(unsigned char * pbin, int bytelen, const char * prefix, int format, int dir ){
// dir=0 > forward; dir=1 -> backward
int c; int bytype;

  if (pbin && bytelen) {   // prevent NULL ptr. and bitlen=0
    bytype = format;
    if(dir<0 || dir>1) {
      printf("printHex(): 'dir' must be 0 or 1\n");
      return;  // check dir-marker range
    }
    printf("%s",prefix);
    for(c=(bytelen-1)*dir; c!=(bytelen*(1-dir)-dir); c=c+1-(2*dir)){
      if(!bytype){
        printf("-");
        bytype = format;
      }
      printf("%02X", (unsigned char)pbin[c]);
      bytype--;
    }
  }
}

//*************************************************************************
// PrintBit()
// Print binary field: pbin[0] <-> pbin[len-1] by given direction in 0/1 digits
// dump format: '0b-bbbbbbbb-bbbbbbbb-...'  e.g.  0b-10010110-10101100 (dir=0)
//                  76543210-76543210                 0x96      0xAC  (forward)
// Print starts where cursor is and no EOL is used.
// INPUT
//    pbin    byte ptr on binary field[0]
//    bitlen  number of bits (!) to be printed in bit stream format
// (0)dir     =0 -> forward  [0...len-1],   =1 -> backward [len-1...0]
// (1)format  =1: bytewise  =2: 2bytes(short)  =4bytes(word) =8bytes(long)...
//*************************************************************************
void Printbit(unsigned char * pbin, int bitlen, const char * prefix, int format, int dir){
int c; int bit; int len;
int bytype;

  if (pbin && bitlen) {   // prevent NULL ptr. and bitlen=0
    if(dir<0 || dir>1) {
      printf("PrintBit(): 'dir' must be 0 or 1\n");
      return;  // check dir-marker range
    }
    len = bitlen/8;     // get byte counter
    if(len*8 < bitlen)  // remaining bits after last full byte ?
      len++;            // we have to do byteloop one more time
    bytype=format;
    printf("%s",prefix);
    for(c=(len-1)*dir; c!=(len*(1-dir)-dir); c=c+1-(2*dir)){      // byte loop forw./backw. 
      if(!bytype){
        printf("-");
        bytype=format;
      }
      for(bit=7*dir; bit!=8*(1-dir)-dir; bit=bit+1-(2*dir)){      // bit loop forw./backw.
        printf("%c", pbin[c] & (1u << bit) ? '1' : '0');
        if(!(--bitlen))   // last requested bit printed ?
          return;
      }
      bytype--;
    }
  }
} // end of Printbit()



//******************************************************************************
// print hexdump of msg[len]  in the format:
// 0xaaaa:  dddd dddd dddd dddd  dddd dddd dddd dddd  <cccccccc cccccccc>
void hexdump(unsigned char * msg, int len){
	int i, y, count;
	unsigned char c;
	
	printf("Address:  0 1  2 3  4 5  6 7   8 9  A B  C D  E F  lenght=%iByte\n", len);
	count = 0;
	while(count < len){
		// print len\16 lines each of 4 x 4 words
		printf("  +%4X: ", (uint32_t)count);
		for(y=0; y<16; y++){
			printf("%02X", (unsigned char) msg[count++]);
			if (count == len)
				break;
			printf("%02X ", (unsigned char) msg[count++]);
			y++;
			if (count == len)
				break;
			if(y==7)
				printf(" ");
		}

		if(y<16){	// break condition reached: end of byte field
			// at this point y-1 bytes already printed (0..15)
			// we have to fill up the line with " "
			i=y;				// remember # of already printed bytes-1
			if((y+1)%2 ==1){	// do we have a split word ?
				printf("   ");	// yes, fill up the gap
				i++;			// one byte less
			}
			for( ; i<15; i++)	// fill up the rest bytes of the line
				printf("  ");
			if(y<7)				// already reached 2. half ?
				printf(" ");	// no, fill up gap
			for(i=0; i<((15-y)*2)/4; i++)	// fill up gap between each word
				printf(" ");

			y++;	// compensate break condition of 'for(; ;y++)'
		}
		// just line end reached -> wrap the line
		// print text representation of each line		
		printf(" <");
		// start with regular letters
		for (i=0; i<y; i++){
			if(i==8) printf(" ");
			c = msg[count-y+i];
			if(c < 32 || c >= 127)
				printf(".");
			else
				printf("%c", (char)c);
		}
		printf(">\n");	// end of full text field print
	}
  return;	
} // end of hexdump()

// end of main.cpp