/*******************************************************************************
* The "BeeIoTWAN_GW" Module is distributed under the New BSD license:
*
* Copyright (c) 2020, Randolph Eser
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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/time.h>
#include <time.h>

#include <sys/ioctl.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <linux/sockios.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>

#include "BeeIoTWan.h"
#include "beeiot.h"

unsigned int	lflags;               // BeeIoT log flag field

/*******************************************************************************
 * BIoTWAN global variables
 * 
 *******************************************************************************/

// Central Database of all measured values and runtime parameters
dataset			bhdb;		// central beeIoT data DB
configuration	cfgini;		// runtime parameter init source
	
// to retrieve local LAN Port MAC address
struct sockaddr_in si_other;
int s, slen = sizeof(si_other);
struct ifreq ifr;

extern struct timeval now;		// current tstamp used each time a time check is done
extern char	  TimeString[128];	// contains formatted Timestamp string of each loop(in main())


//******************************************************************************
//  Function prototypes
extern int  NwNodeScan (void);


/******************************************************************************
 * MAIN()
 ******************************************************************************/
int main () {
    struct timeval nowtime;
    uint32_t lasttime;
	int rc;

	
	setbuf(stdout, 0);		// disable buffering on stdout
	printf("*** BeeIoT-WAN MAIN ***\n");

//  lflags = 0;   // Define Log level (search for Log values in beeiot.h)
//  lflags = LOGBH + LOGOW + LOGHX + LOGLAN + LOGEPD + LOGSD + LOGADS + LOGSPI + LOGLORAR + LOGLORAW;
	lflags = LOGBH + LOGLORAW + LOGLORAR;

	// get current timestamp
	gettimeofday(&now, 0);
	strftime(TimeString, 80, "%d-%m-%y %H:%M:%S", localtime(&now.tv_sec));
	 BHLOG(LOGBH) printf("%s: Setup started with V%d.%d.%d\n", 
		TimeString,  (int)BIoT_VMAJOR, (int)BIoT_VMINOR, (int)BIoT_VMINSUB);


	int fd = wiringPiSetup () ;
    if(fd== -1){
		printf("  Wiring Pi Setup failed: %i", fd);
		exit(-1);
	}

	// Get Local LAN Port settings
    if ( (s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
        printf("  No LAN socket detectd !!!\n");	// if no LAN Port -> do nothing
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
	
		// preset some local config settings
	strcpy(cfgini.web_root, "/var/www/html/beeiot");
	strcpy(cfgini.bh_CSVFILE, "beeiotlog");		// results in beeiotlogYYYY.csv

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

	// run forever: wait for incoming packages via radio_irq_handler()
	// STart BIoT WAN Node scan routine
	NwNodeScan ();
	// will never reach this point !
	// Further processing of BIoT messages by BIoTApp() function

    return (0);
}



// end of main()
