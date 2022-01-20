/*******************************************************************************
* The "JoinSrv" Module is distributed under the New BSD license:
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
* home page for more info: https://github.com/beeiot
********************************************************************************
*/

/*******************************************************************************
 * Sub Module JoinSrv.cpp
 *
 *******************************************************************************
 */

#include <iostream>
#include <cstdint>
#include <cstring>
#include <sys/time.h>

#include "base64.h"

#define CHANNELTAB_EXPORT // enable channelcfg tab instance here: channeltable_t	txchntab[MAX_CHANNELS]
#include "BeeIoTWan.h"
#undef CHANNELTAB_EXPORT  // block double instanciation

#include "beeiot.h"
#include "gwqueue.h"	// using STL container classes
#include "regslora.h"
#include "beelora.h"
#include "radio.h"
#include "JoinSrv.h"
#include "BIoTApp.h"
#include "BIoTCrypto.h"

//******************************************************************************
// Global Logging flags + User runtime settings
extern unsigned int		lflags;		// BeeIoT log flag field
extern configuration	*cfgini;	// ptr. to struct with initial user params

//******************************************************************************
// API to BIoTApp.cpp
extern int AppBIoT		(int ndid, char* data, byte len, int mid);	// Bee Weight Scale App
extern int AppTurtle	(int ndid, char* data, byte len, int mid);	// Turtle House Control App
extern int AppGH		(int ndid, char* data, byte len, int mid);	// GreenHouse Control App


//******************************************************************************
// WLTab[]
// Global Node Reference table for JOIN validation
// Nodes not registered here are not served at all (even not at JOIN)
// Don't change entry 0 line ! (Keep last Zero line as template)
// => The hit index position in this table results to NodeID = NODEIDBASE + idx !
// NODEIDBASE+0 to be used by new node for JOIN communication -> else rejected !

static nodewltable_t WLTab[MAXNODES+2]={			// +2 for dummy JOIN lines ID=0,n
// Active entries may get defined/overwritten by JoinSrv constructor with cfgini-Data !

// 0: Dummy start marker of table 
// (NODEID == 0x00 -> used for JOIN requests only => don't change)
	NODEIDBASE, GWID0, 0,  0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,  10,0,0,0,0x00,
//---------------------------------------------------
// 1: BeeIoT ESP32-WROOM32:	MAC: 24:6F:28:D5:8A:DC	// default: BeeHive Weightcell #1
	NODEID1, GWID1,	0, BIoT_EUID,					// ndid, gwid, mid, AppEUI: BIoT
	0xDC, 0x8A, 0xD5, 0xFF, 0xFE, 0x28, 0x6F, 0x24, // DevEUI 
	10, 0, 0, 0, 0x00,								// reportfrq, joinflag, chncfg, hwconfig
//---------------------------------------------------
// 2: ESP32-WROVERB: MAC 24:6F:28:F0:0D:AC			// beacon test Module 1
	NODEID2, GWID1,	0, BIoT_EUID,					// ndid, gwid, mid, AppEUI: BIoT
	0xAC, 0x0D, 0xF0, 0xFF, 0xFE, 0x28, 0x6F, 0x24, // DevEUI
	1, 0, 0, 0, 0x00,								// reportfrq, joinflag, chncfg, hwconfig
//---------------------------------------------------
// 3: BeeIoT ESP32-WROOM32:	MAC: 94:FE:8A:B5:AA:8C	// Beacon test Module 2
	NODEID3, GWID1,	0, BIoT_EUID,					// ndid, gwid, mid, AppEUI: BIoT
	0x94, 0xFE, 0x8A, 0xFF, 0xFE, 0xB5, 0xAA, 0x8C, // DevEUI 
	1, 0, 0, 0, 0x00,									// reportfrq, joinflag, chncfg, hwconfig
//---------------------------------------------------
// 4: BeeIoT ESP32-WROOM32:	MAC: 2C:2B:16:28:6F:24 	// Beehive Weight cell test Module 3
	NODEID4, GWID2,	0, BIoT_EUID,					// ndid, gwid, mid, AppEUI: BIoT
	0x2C, 0x2B, 0x16, 0xFF, 0xFE, 0x28, 0x6F, 0x24, // DevEUI 
	10, 0, 0, 0, 0x00,								// reportfrq, joinflag, chncfg, hwconfig
//---------------------------------------------------
// 5: fill in more nodes here ... ( but add in config.ini as well)
//---------------------------------------------------
// N: Dummy end marker of table (NODEID == 0x00)
	0x00, 0x00, 0x00, 0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,  0,0,0,0,0x00,
//---------------------------------------------------
};

uint8_t  NwSKey[16]	= {0xDD, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};

#define  NAPPID	4
byte  appid[NAPPID][LENJOINEUI] = {
	BIoT_EUID,		// BeeIot Weight cell
	TURTLE_EUID,	// Turtle House monitoring
	GH_EUID,		// Plant house  monitoring
	0,0,0,0,0,0,0,0,
};


//***************************************************************************
// JoinSrv Constructor
JoinSrv::JoinSrv(gwbind_t &gwtab, int nmodem): gwt(gwtab), mactive(nmodem){
	nodedb_t * pndb;

	JS_Cfg2Wlt();				// update WLTab[] by cfgini settings
	
	// Preset all NDB[] entries
	// Init NodeDB[] for new node registrations:
	for(int i=0; i<MAXNODES; i++){
		pndb = &NDB[i];

		// nodeinfo params will be set by data of JOIN request package of node
		pndb->nodeinfo.vmajor		= 0;	// free entry if 'NDB[x].nodeinfo.vmajor == 0'
		pndb->nodeinfo.vminor		= 0;
		pndb->nodeinfo.devEUI[0]	= 0;
		pndb->nodeinfo.devEUI[1]	= 0;
		pndb->nodeinfo.devEUI[2]	= 0;
		pndb->nodeinfo.devEUI[3]	= 0;
		pndb->nodeinfo.frmid[0]		= 0;
		pndb->nodeinfo.frmid[1]		= 0;		
		// Default MID: init by RegisterNode (from WL-Table)
		pndb->middef				= 0;	// this mid is not used for communication /w node: use msg.mid
											// because GW answers always in slave mode on received msg

		pndb->nodecfg.gwid			= WLTab[i].gwid;
		pndb->nodecfg.nodeid		= WLTab[i].nodeid;
		pndb->nodecfg.vmajor		= BIoT_VMAJOR;		// Major + Minor version: Vx.y
		pndb->nodecfg.vminor		= BIoT_VMINOR;
		pndb->nodecfg.verbose		= lflags;			// finally set by CONFIG command
		pndb->nodecfg.nonce			= 0;
		pndb->nodecfg.hwconfig		= WLTab[i].hwconfig;

		pndb->msg.ack				= 0;
		pndb->msg.pkgid				= 0;
		pndb->msg.retries			= 0;
		pndb->msg.rssi				= 0;
		pndb->msg.snr				= 0;	

		if (WLTab[i].nodeid == 0){	// end of pre init field for WLTab reached ?
			memset(&WLTab[i], 0, sizeof(nodewltable_t));	// reset all remaining fields
		}
		pndb->pwltab				= &WLTab[i];	// link to WLTAB[] of node i
	}
	memset(&WLTab[MAXNODES], 0, sizeof(nodewltable_t));	// reset last entry in WLTab[]

	// set JOIN default modem for NDB-ID: 0 -> node ID = NODEIDBASE
	NDB[0].middef = gwtab.joindef;	
} // end of JoinSrv()

// JoinSrv Destructor
JoinSrv::~JoinSrv(void){}

//***************************************************************************
// JS_Cfg2Wlt()
// - copy cfgini dataset of GW settings to corresponding WLTab[] entry 
// This function should be called anytime cfgini was parsed again to keep WLTab up to date.
// INPUT:
//	pkg		ptr to BIoTWAN JOIN Cmd package from node in question
// RETURN:
//	nodeid	index of newly registered or already known node at NodeDB[nodeid]
//  -1		registration failed (unknown: DevID)
//  -2		registration failed (NodeDB full) -> MAXDEVID reached
//***************************************************************************

void JoinSrv::JS_Cfg2Wlt(void){
	int nid, i;
	byte * p1;
	byte * p2;
	
	lflags	= (unsigned int) cfgini->biot_verbose;	// get the custom verbose mode

	// Update WLTab[] entries by pconfig data
	nid = 1;
	WLTab[nid].nodeid	= NODEIDBASE + nid;
	WLTab[nid].gwid		= GWIDx-cfgini->nd1_gwid;
	WLTab[nid].mid		= cfgini->nd1_mid;
	WLTab[nid].chncfg	= gwt.gwset[WLTab[nid].mid]->chncfgid;	// get Radio channel cfg of current Gateway
	WLTab[nid].reportfrq= cfgini->nd1_freport;			// status report frequency in Min.
	WLTab[nid].wcalib	= cfgini->nd1_wcalib;			// 16bit weight calibration value
	WLTab[nid].hwconfig = cfgini->nd1_hwconfig;			// get HW component enable flags (8bit)
	NDB[nid].msg.mid			= WLTab[nid].mid;		// preset MID even /wo any prev. package
	NDB[nid].nodecfg.channelidx	= WLTab[nid].chncfg;	// cfg.-channelid of modem
	NDB[nid].nodecfg.freqsensor	= WLTab[nid].reportfrq; // [min] reporting frequence of status pkg.
	NDB[nid].wcalib				= WLTab[nid].wcalib;	// get static Weight cell calib value for daily work

	if(cfgini->nd1_appeui > NAPPID)
		cfgini->nd1_appeui = 1;
	memcpy(&WLTab[nid].AppEUI, &appid[cfgini->nd1_appeui-1][0], LENJOINEUI);

	p1= (byte *) &cfgini->nd1_deveuiup;
	p2= (byte *) &cfgini->nd1_deveuilo;
	for (i=0;i<4; i++){
		WLTab[nid].DevEUI[i]	= (byte) p1[3-i];
		WLTab[nid].DevEUI[4+i]	= (byte) p2[3-i];
	}
	
	nid = 2;
	WLTab[nid].nodeid	= NODEIDBASE + nid;
	WLTab[nid].gwid		= GWIDx-cfgini->nd2_gwid;
	WLTab[nid].mid		= cfgini->nd2_mid;
	WLTab[nid].chncfg	= gwt.gwset[WLTab[nid].mid]->chncfgid;	// get Radio channel cfg of current Gateway
	WLTab[nid].reportfrq= cfgini->nd2_freport;
	WLTab[nid].wcalib	= cfgini->nd2_wcalib;
	WLTab[nid].hwconfig = cfgini->nd2_hwconfig;			// get HW component enable flags (8bit)
	NDB[nid].msg.mid			= WLTab[nid].mid;		// preset MID even /wo any prev. package
	NDB[nid].nodecfg.channelidx	= WLTab[nid].chncfg;	// cfg.-channelid of modem
	NDB[nid].nodecfg.freqsensor	= WLTab[nid].reportfrq; // [min] reporting frequence of status pkg.
	NDB[nid].wcalib				= WLTab[nid].wcalib;	// get static Weight cell calib value for daily work

	if(cfgini->nd2_appeui > NAPPID)
		cfgini->nd2_appeui = 1;
	memcpy(&WLTab[nid].AppEUI, &appid[cfgini->nd2_appeui-1][0], LENJOINEUI);
	p1= (byte *) &cfgini->nd2_deveuiup;
	p2= (byte *) &cfgini->nd2_deveuilo;
	for (i=0;i<4; i++){
		WLTab[nid].DevEUI[i]	= (byte) p1[3-i];
		WLTab[nid].DevEUI[4+i]	= (byte) p2[3-i];
	}
	
	nid = 3;
	WLTab[nid].nodeid	= NODEIDBASE + nid;
	WLTab[nid].gwid		= GWIDx-cfgini->nd3_gwid;
	WLTab[nid].mid		= cfgini->nd3_mid;
	WLTab[nid].chncfg	= gwt.gwset[WLTab[nid].mid]->chncfgid;	// get Radio channel cfg of current Gateway
	WLTab[nid].reportfrq= cfgini->nd3_freport;
	WLTab[nid].wcalib	= cfgini->nd3_wcalib;
	WLTab[nid].hwconfig = cfgini->nd3_hwconfig;			// get HW component enable flags (8bit)
	NDB[nid].msg.mid			= WLTab[nid].mid;		// preset MID even /wo any prev. package
	NDB[nid].nodecfg.channelidx	= WLTab[nid].chncfg;	// cfg.-channelid of modem
	NDB[nid].nodecfg.freqsensor	= WLTab[nid].reportfrq; // [min] reporting frequence of status pkg.
	NDB[nid].wcalib				= WLTab[nid].wcalib;	// get static Weight cell calib value for daily work

	if(cfgini->nd3_appeui > NAPPID)
		cfgini->nd3_appeui = 1;
	memcpy(&WLTab[nid].AppEUI, &appid[cfgini->nd3_appeui-1][0], LENJOINEUI);
	p1= (byte *) &cfgini->nd3_deveuiup;
	p2= (byte *) &cfgini->nd3_deveuilo;
	for (i=0;i<4; i++){
		WLTab[nid].DevEUI[i]	= (byte) p1[3-i];
		WLTab[nid].DevEUI[4+i]	= (byte) p2[3-i];
	}
	
	nid = 4;
	WLTab[nid].nodeid	= NODEIDBASE + nid;
	WLTab[nid].gwid		= GWIDx-cfgini->nd4_gwid;
	WLTab[nid].mid		= cfgini->nd4_mid;
	WLTab[nid].chncfg	= gwt.gwset[WLTab[nid].mid]->chncfgid;	// get Radio channel cfg of current Gateway
	WLTab[nid].reportfrq= cfgini->nd4_freport;
	WLTab[nid].wcalib	= cfgini->nd4_wcalib;
	WLTab[nid].hwconfig = cfgini->nd4_hwconfig;			// get HW component enable flags (8bit)
	NDB[nid].msg.mid			= WLTab[nid].mid;		// preset MID even /wo any prev. package
	NDB[nid].nodecfg.channelidx	= WLTab[nid].chncfg;	// cfg.-channelid of modem
	NDB[nid].nodecfg.freqsensor	= WLTab[nid].reportfrq; // [min] reporting frequence of status pkg.
	NDB[nid].wcalib				= WLTab[nid].wcalib;	// get static Weight cell calib value for daily work

	if(cfgini->nd4_appeui > NAPPID)
		cfgini->nd4_appeui = 1;
	memcpy(&WLTab[nid].AppEUI, &appid[cfgini->nd4_appeui-1][0], LENJOINEUI);
	p1= (byte *) &cfgini->nd4_deveuiup;
	p2= (byte *) &cfgini->nd4_deveuilo;
	for (i=0;i<4; i++){
		WLTab[nid].DevEUI[i]	= (byte) p1[3-i];
		WLTab[nid].DevEUI[4+i]	= (byte) p2[3-i];
	}
	
	nid = 5;
	WLTab[nid].nodeid	= 0;	// flag table end here
	WLTab[nid].gwid		= 0;
	WLTab[nid].mid		= 0;
	WLTab[nid].reportfrq= 0;
	WLTab[nid].joined	= 0;
	WLTab[nid].hwconfig	= 0;

} // end of JS_Cfg2Wlt()



//***************************************************************************
// JS_RegisterNode()
// - check if node is already registered in NodeDB or known in DevID list (const)
// - create new noder registration entry in NodeDB[devid]
// - setup Node cfg runtime params in NodeDB[devid] => for CMD_CONFIG action
// INPUT:
//	pkg		ptr to BIoTWAN JOIN Cmd package from node in question
// RETURN:
//	nodeid	index of newly registered or already known node at NodeDB[nodeid]
//  -1		registration failed (unknown: DevID)
//  -2		registration failed (NodeDB full) -> MAXDEVID reached
//***************************************************************************

int JoinSrv::JS_RegisterNode(beeiotpkg_t * pkg){
// default = sync mode -> async = 0
beeiot_join_t * pjoin;  // ptr. on JOIN message format field (reusing pkg buffer)
nodewltable_t * pwltab;	// ptr. for walking through the WL Table
nodedb_t	  * pndb;	// ptr for update of nodedb[]
byte ndid =0;
int  rc =0;

	pjoin = (beeiot_join_t*) pkg; // map generic packet to CMD_JOIN/CMD_REJOIN type 
	BHLOG(LOGLORAW) printf("  RegisterNode: search WLTable[]\n");

	for(ndid=1; ndid < MAXNODES; ndid++){
		pwltab = & WLTab[ndid];	//0..MAXNODES-1
		if(pwltab->nodeid ==0){ // reached end of "initialized" table ?
			BHLOG(LOGLORAW) printf("  RegisterNode: Node not registered in WLTable\n");
			rc=-1;
			return(rc);	// nothing more to do => DevEUI unknown -> have to reject this join request
		}
		BHLOG(LOGLORAW) printf("    Cmp[%d] ", (unsigned char) ndid);
		BHLOG(LOGLORAW) Printhex((byte*)& pjoin->info.devEUI,  8, "  Pkg-DEVEUI: 0x-"); 
		BHLOG(LOGLORAW) Printhex((byte*)& pwltab->DevEUI,  8, "  WL-DEVEUI: 0x-"); 
		BHLOG(LOGLORAW) printf("\n");
		if(JS_ByteStreamCmp((byte *) &pjoin->info.devEUI, (byte *) &pwltab->DevEUI,8) == 0){
			rc = ndid;
			break; // we have a hit -> known DevEUI found
		}
		rc=-1;	// for this entry we have no hit, try next one
	}
	if (ndid == MAXNODES){	// reached end of table ?
		BHLOG(LOGLORAW) printf("  RegisterNode: Node not registered till WLTable-End\n");
		return(rc); // nothing more to do => DevEUI unknown -> have to reject this join request
	}
	if(pwltab->joined){		// already joined known node ?	-> was a rejoin ?
		// assumed NDB[] was already initialized with this node till last session
		pndb = & NDB[ndid];							// get pointer to already initialized NDB entry
		BHLOG(LOGLORAW) printf("  RegisterNode: Node found in WLTable[%d] -> joined on modem %i (chn%i,HWCfg:%d)\n",
				(int) ndid, pndb->middef, pndb->nodecfg.channelidx, pndb->nodecfg.hwconfig);
		pndb->nodecfg.channelidx= pwltab->chncfg;	// but we start with default Channel IDX again
		pndb->middef = pwltab->mid;					// and default modem
		
		// may be node wants to rejoin to another AppID
		memcpy(&pndb->nodeinfo.joinEUI, &pjoin->info.joinEUI, 8);

		// may be FW was updated in between
		pndb->nodeinfo.vmajor	= pjoin->info.vmajor;	// get Version of BIoT protocol of node
		pndb->nodeinfo.vminor	= pjoin->info.vminor;

		pndb->nodecfg.nonce		= pjoin->hd.pkgid;
		// ToDo split FrameIdx and PkgIdx
		pndb->nodeinfo.frmid[1]	= pjoin->hd.pkgid;		// by now FrameIdx and Pkg Idx are identical
		pndb->msg.pkgid			= pjoin->hd.pkgid;		// keep using current counter status as devnonce;
		pndb->nodecfg.freqsensor= pwltab->reportfrq;// [min] loop time of sensor status reports in Seconds (from config.ini)
		return(ndid);	// return index to NDB[]
	}

	// Else, use same nodeid of WL-Table for NDB -> should be always a free entry in NodeDB
	pndb = & NDB[ndid];
	
	// now we can update WLTable & NDB for new joined node:
	BHLOG(LOGLORAW) printf("  RegisterNode: DevEUI registered -> update NodeDB[%i] with NodeID: 0x%02X, for Modem%i\n",
			(unsigned char)ndid, (unsigned char)pwltab->nodeid, (unsigned char)pwltab->mid );

	pwltab->joined = 1;				// mark WL Table entry as "joined"
	// initialize NDB entry:
	memcpy(&pndb->nodeinfo.devEUI, &pwltab->DevEUI,8);
	memcpy(&pndb->nodeinfo.joinEUI, &pjoin->info.joinEUI, 8);
	pndb->nodeinfo.frmid[0] = 0;			// MSB: init "last frame msg id"
	pndb->nodeinfo.frmid[1] = 0;			// LSB:  -> First Session to be started with 0x01
	pndb->nodeinfo.vmajor	= pjoin->info.vmajor;	// get Version of BIoT protocol of node
	pndb->nodeinfo.vminor	= pjoin->info.vminor;

	pndb->nodecfg.verbose	= (u2_t)lflags;			// get same GW verbose mode for client too
	pndb->nodecfg.channelidx= pwltab->chncfg;;		// we start with default Channel IDX
	pndb->nodecfg.gwid		= pwltab->gwid;			// store predefined GW (ModemID = gwid - GWIDx)
	pndb->nodecfg.nodeid	= pwltab->nodeid;
	pndb->nodecfg.freqsensor= pwltab->reportfrq;	// [min] loop time of sensor status reports in Seconds (from config.ini)
	pndb->nodecfg.vmajor	= pjoin->info.vmajor;	// get Version of BIoT protocol of node
	pndb->nodecfg.vminor	= pjoin->info.vminor;	// gives room for backward support stepping
	pndb->nodecfg.nonce		= pndb->nodeinfo.frmid[1];	// init packet index by LSB of FrmID
	pndb->nodecfg.hwconfig	= pwltab->hwconfig;		// init HW configuration of node
	// date and time will be defined dynamic at each JOIN ACK creation by BIoTFlow()
	pndb->msg.ack			= 0;
	pndb->msg.retries		= 0;
	pndb->msg.pkgid			= 0;	// last msg idx => no msg received yet
	// FrmID & PkgID are handled/checked identical !
	pndb->msg.mid			= 0;	// pkg-mid: will be redefined by each new package on which channel it came in
	pndb->middef			= pwltab->mid;	// get default Modem ID from WL Table; used by ???
	pndb->pwltab			= pwltab;	// back link to WL Table

	// Values T.b.d
	pndb->DevAddr = ndid;	// better than 0;
	pndb->AppSKey[0] = 0;
	pndb->NwSKey[0]  = 0;

    BHLOG(LOGBH) printf("  Node Joined now!  -> assigned: GWID:0x%02X, NodeID:0x%02X, HWCfg:%d ", 
				(unsigned char) pwltab->gwid, (unsigned char)pwltab->nodeid, pndb->nodecfg.hwconfig);
	BHLOG(LOGLORAW) Printhex((byte*)& pndb->DevAddr, 4,  "DevAddr: 0x", 4); 
	BHLOG(LOGLORAW) printf("\n");
    BHLOG(LOGLORAW) Printhex( pndb->nodeinfo.devEUI,  8, "    DEVEUI: 0x-"); 
	BHLOG(LOGLORAW) printf("\n");
    BHLOG(LOGLORAW) Printhex( pndb->nodeinfo.joinEUI, 8, "   JOINEUI: 0x-"); 
	BHLOG(LOGLORAW) printf("\n");
    BHLOG(LOGLORAW) Printhex( pndb->AppSKey, 16,         "   AppSKEY: 0x-", 2);
	BHLOG(LOGLORAW) printf("\n");
    BHLOG(LOGLORAW) Printhex( pndb->NwSKey,  16,         "    NwSKEY: 0x-", 2);
	BHLOG(LOGBH) printf("\n");

	return(ndid);
}

//***************************************************************************
// JoinSrv_ValidateNode()
// Check Pkg-Header based on NDB Data
// 1. Check MIC based pkg integrity
// 2. Search NDB for mutual NodeID
// 3. Check corresponding GW ID
// 4. Check WLTab for registered but not joined Nodes
// INPUT:
//	mystatus	ptr to raw BIoTWAN pkg
// RETURN:
// >=0	NDBidx of found entry
// -1	wrong GW ID
// -2	wrong NodeID
// -3	Known in WLTab but not joined -> no NDB entry
// -4   Known and Joined, but using wrong GWID -> should rejoin
// -5
// -6	Pkg Header o.k., but payload data corrupted
//
int JoinSrv::JS_ValidatePkg(beeiotpkg_t* mystatus){
	int rc =0;
	int ndid;

	// 1. check range of mutual GWID and NodeID (incl. JOIN requests !)
	if((mystatus->hd.sendID < NODEIDBASE) || (mystatus->hd.sendID > NODEIDBASE+MAXNODES)){
		// This is no known NodeID for this curr. Nw server instance !
		rc=-1;
		return(rc);
	}
	// 2. Pkg GW ID in range: (GWIDx-MAXGW-1)...GWIDx ?
	if((mystatus->hd.destID >= GWIDx) && (mystatus->hd.destID < (GWIDx-MAXGW))){ 
		// This is no packet for this current Nw server instance !
		rc = -2;
		return(rc);
	}

	
	// 4. Compare Pkg Header with corresponding WLTab[] and NDB[] entries
	ndid = mystatus->hd.sendID - NODEIDBASE;	// extract mutual NDB index
	if(!WLTab[ndid].joined){ // This Node is known/registered but not joined yet ?
		// Not joined &  CMD==JOIN: NODEIDBASE-ID request 
		if( mystatus->hd.sendID == NODEIDBASE && mystatus->hd.cmd == CMD_JOIN ){
				// Validate Pkg with Pkg-MIC integrity check
				// ... Also for JOIN requests
				if(JS_ValidateMic(mystatus, CMD_JOIN, ndid) < 0){ // Pkg Integrity by MIC o.k. ?
					// PKG payload data is corrupted !
					rc=-3;	// JOIN pkg corrupt -> initiate a rejoin request
					return(rc);
				}
			// o.k. this is already a valid JOIN request -> lets do it by usual parse function...
			return(0);
		}
		// Known but not joined -> Node should initiate a JOIN request first -> rejected
		// Not joined &  any CMD: any Node ID allowed but new JOIN requested (e.g. GW was restarted)
		rc=-3;
		return(rc);
	}
	// 5. Node is joined already -> a fully defined NDB[] is expected
	// take ndid as valid by now
	if( (mystatus->hd.cmd == CMD_REJOIN) ||
		(mystatus->hd.cmd == CMD_JOIN ) ){
		//    Hint: A (RE-)JOIN PKG is bypassed as normal CMD package 
		// -> will be parsed later by detected nodeid
		if(mystatus->hd.destID != (GWIDx-gwt.joindef)){ // calculate user-default JOIN channel
			// No negative GWID check for RE-/JOIN: request will be accepted anyhow, just a warning
			// RE-/JOIN is allowed here via all GWIDs but response is sent by Default-GW to retrieved Client ID only.
			// for cleaner process: Node should always use GWIDx also for RE-/JOIN
			BHLOG(LOGLORAW) printf("  JS_ValidateNode: Warning: Wrong GWID used for already joined Node: 0x%02X\n", (unsigned char) mystatus->hd.destID);
		}
			// Validate Pkg with Pkg-MIC integrity check
			// ... Also for RE-/JOIN requests
			if(JS_ValidateMic(mystatus, CMD_JOIN, ndid) < 0){ // Pkg Integrity by MIC o.k. ?
				// PKG data is corrupted !
				rc=-3;	// JOIN pkg corrupt -> initiate a rejoin request
				return(rc);
			}
			// o.k. Node is already joined and requests reactivation -> lets do it...
			return(0);	// bypass to the JOIN process by BIOTPARSE -> RegisterNode() will find node again
	}
	
	// 6. No (RE-)JOIN package...then GW-ID must fit to NDB assignment
	if(NDB[ndid].nodecfg.gwid != mystatus->hd.destID){
		// this joined node is (still) not using the assigned GWID
		// pkg rejected -> request a rejoin needed
		BHLOG(LOGLORAW) printf("  JS_ValidateNode: joined node is (still) not using the assigned GWID (0x%02X) -> should rejoin\n", (unsigned char) mystatus->hd.destID);
		rc=-4;
		return(rc);
	}
	
	// 7. BIoT-Nw addresses assured now: "This was a package for us" -> now we could answer in any case

	// This check normally to be done by AppServer:
	// Check User Status data length
	if(mystatus->hd.frmlen > BIoT_FRAMELEN){
		BHLOG(LOGLORAW) printf("  JS_ValidateNode: Wrong package size detected (%i) -> retry requested\n", (unsigned char) mystatus->hd.frmlen);		
		// for test purpose: dump payload in hex format
		BHLOG(LOGLORAR) hexdump((unsigned char*) mystatus, MAX_PAYLOAD_LENGTH);
		BHLOG(LOGLORAR) printf("\n");
		rc=-5;
		return(rc);
	}
	
	// 8. Now we have a valid BeeIoT Package header: check the "cmd" for next BeeIoTWAN action
	// Validate Pkg-data with Pkg-MIC integrity check
	if(JS_ValidateMic(mystatus, -1, ndid) < 0){ // Pkg Integrity by MIC o.k. ?
		// PKG data is corrupted !
		rc=-6;	// JOIN pkg corrupt -> initiate a rejoin request
		return(rc);
	}

	// some debug output: assumed all is o.k.
		BHLOG(LOGLORAR) printf("  JS_ValidateNode(0x%02X>0x%02X)", (unsigned char)mystatus->hd.sendID, (unsigned char)mystatus->hd.destID);
		BHLOG(LOGLORAR) printf("[%2i]:(cmd:%02d)", (unsigned char) mystatus->hd.pkgid, (unsigned char) mystatus->hd.cmd);
		int len = mystatus->hd.frmlen;
		if(len >16) {
			len=16;
			BHLOG(LOGLORAR) Printhex((unsigned char*)mystatus->data, len, "<", 2);
			BHLOG(LOGLORAR) printf(" ...> (len=%i)\n", mystatus->hd.frmlen);
		}else{
			BHLOG(LOGLORAR) Printhex((unsigned char*)mystatus->data, len,"<", 2);
			BHLOG(LOGLORAR) printf(">\n");
		}

	return(ndid);	// everything fine with this PKG -> return assigned ndid 
}


//***************************************************************************
// JoinSrv_ValidateMic()
// Check Pkg-integrity by delivered MIC
// 1. Create MIC based on known pkg & client session attributed
// 2. Compare created MIC with given MIC (last bytes of pkg. data)
// INPUT:
//	mystatus	ptr to raw BIoTWAN pkg
//  mode		=0 Compute data MIC
//				=CMD_JOIN/CMD_REJOIN compute JOIN MIC
// ndid			Node ID of NDB-Tab entry
// RETURN:
// >=0	pkg integrity o.k.
// -1	pkg data corrupted
//
int JoinSrv::JS_ValidateMic(beeiotpkg_t* mystatus, uint8_t mode, int ndid){
int rc =0;
uint32_t bufmic = 0x11223344;
uint32_t * pduid;
byte * pmic;
beeiot_join_t* pjoin;

  // ToDo get MIC calculation:
  // byte cmac[16]={0x11,0x22,0x33,0x44};
  // cmac = aes128_cmac(NwkKey, MHDR | JoinEUI | DevEUI | DevNonce)
  if(mode == CMD_JOIN || mode == CMD_REJOIN){
    // For JoinPkg:
	 pjoin = (beeiot_join_t*) mystatus;
    LoRaMacJoinComputeMic( (const uint8_t*) mystatus, (uint16_t) mystatus->hd.frmlen+BIoT_HDRLEN,
               (const uint8_t*) &NwSKey , (uint32_t*) &bufmic );
  }else{
    // For Upload Pkg:
	pduid = (uint32_t*) &(WLTab[ndid].DevEUI);
	LoRaMacComputeMic( (const uint8_t*)mystatus, (uint16_t) mystatus->hd.frmlen+BIoT_HDRLEN, (const uint8_t*) &NwSKey,
             (const uint32_t) *pduid, UP_LINK, (uint32_t) mystatus->hd.pkgid, (uint32_t*) &bufmic );
  }

	pmic = (byte*) &(mystatus->data);
	pmic += mystatus->hd.frmlen;
	
	BHLOG(LOGLORAW) printf("  JS_ValidateMic: Pkg-MIC[4]: 0x%02X%02X%02X%02X == 0x%X ", 
      pmic[0], pmic[1],  pmic[2], pmic[3], bufmic);

  	if(	((bufmic >>24) & 0xFF) == pmic[0] &&
		((bufmic >>16) & 0xFF) == pmic[1] &&
		((bufmic >> 8) & 0xFF) == pmic[2] &&
		((bufmic     ) & 0xFF) == pmic[3]){
		// MIC is correct
		BHLOG(LOGLORAW) printf(" -> Pkg.-data o.k. !\n");
		return(0);
	}

	BHLOG(LOGLORAW) printf(" -> Pkg.-data corrupted !\n");
  
	return(-1);
//	return(0);	// for test purpose
}
