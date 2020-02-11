/*******************************************************************************
* The "JoinSrv" Module is distributed under the New BSD license:
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
* home page for more info: https://github.com/beeiot
********************************************************************************
*/

/*******************************************************************************
 * Sub Module JoinSrv.cpp
 *
 *******************************************************************************
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <iostream>
#include <sys/time.h>

#include "base64.h"
#include "loraregs.h"

#define CHANNELTAB_EXPORT // enable channelcfg tab instance here: channeltable_t	txchntab[MAX_CHANNELS]
#include "BeeIoTWan.h"
#undef CHANNELTAB_EXPORT  // block double instanciation

#include "beelora.h"
#include "beeiot.h"


//******************************************************************************
// central Logging flags
extern unsigned int	lflags;               // BeeIoT log flag field

nodewltable_t WLTab[MAXNODES+1]={ // +1 for dummy JOIN line ID=0
// => The index position in this table results in corresponding NodeID: NODEIDBASE+idx !
// 0: Dummy start marker of table (NODEID == 0x00 -> used for JOIN requests => don't change)
	0x00,     0x00, 0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,  0,0,
//---------------------------------------------------
// 1: BeeIoT Prototype 1:	DC8AD5286F24
	NODEID1, GWID1,	
	0xBB, 0xEE, 0xEE, 0xBB, 0xEE, 0xEE, 0x00, 0x00,	// AppEUI
	0xDC, 0x8A, 0xD5, 0xFF, 0xFE, 0x28, 0x6F, 0x24, // DevEUI 
	10, 0,											// reportfrq, joinflag
//---------------------------------------------------
// 2: WROVER ESP32
	NODEID2, GWID1,	
	0xBB, 0xEE, 0xEE, 0xBB, 0xEE, 0xEE, 0x00, 0x00, // AppEUI
	0xAC, 0x0D, 0xF0, 0xFF, 0xFE, 0x28, 0x6F, 0x24, // DevEUI
	1, 0,											// reportfrq, joinflag
//---------------------------------------------------
// 3: fill in more nodes here ...
//---------------------------------------------------
// N: Dummy end marker of table (NODEID == 0x00)
	0x00, 0x00,  0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,  0,0,
};//-------------------------------------------------


nodedb_t NDB[MAXNODES];	// if 'NDB[x].nodeinfo.version == 0' => empty entry
// -> typeset see beelora.h

//******************************************************************************
// local Function prototypes
int RegisterNode	(beeiotpkg_t * joinpkg, int async);
int JS_ValidatePkg	(beeiotpkg_t* mystatus);
int	ByteStreamCmp	(byte * bina, byte * binb, int binlen);


//***************************************************************************
// RegisterNode()
// - check if node is already registered in NodeDB or known in DevID list (const)
// - create new noder registration entry in NodeDB[devid]
// - setup Node cfg runtime params in NodeDB[devid] => for CMD_CONFIG action
// In: async =1 return immediately; =0 wait for TXDone flag 
// Return:
//	nodeid	index of newly registered or already known node at NodeDB[nodeid]
//  -1		registration failed (unknown: DevID)
//  -2		registration failed (NodeDB full) -> MAXDEVID reached
//***************************************************************************

int RegisterNode(beeiotpkg_t * pkg, int async){
// default = sync mode -> async = 0
beeiot_join_t * pjoin;  // ptr. on JOIN message format field (reusing pkg buffer)
nodewltable_t * pwltab;	// ptr. for walking through the WL Table
nodedb_t	  * pndb;	// ptr for update of nodedb[]
byte ndid =0;
int  rc =0;

	pjoin = (beeiot_join_t*) pkg; // map generic packet to CMD_JOIN type 
	BHLOG(LOGLORAW) printf("  RegisterNode: search WLTable[]\n");
	for(ndid=1; ndid < MAXNODES; ndid++){
		pwltab = & WLTab[ndid];	//0..MAXNODES-1
		if(pwltab->nodeid ==0){ // reached end of "initialized" table ?
			BHLOG(LOGLORAW) printf("  RegisterNode: Node not registerd in WLTable\n");
			rc=-1;
			return(rc);	// nothing more to do => DevEUI unknown -> have to reject this join request
		}
		BHLOG(LOGLORAW) printf("    Cmp[%d] ", ndid);
		BHLOG(LOGLORAW) Printhex((byte*)& pjoin->info.devEUI,  8, "  Pkg-DEVEUI: 0x-"); 
		BHLOG(LOGLORAW) Printhex((byte*)& pwltab->DevEUI,  8, "  WL-DEVEUI: 0x-"); 
		BHLOG(LOGLORAW) printf("\n");
		if(ByteStreamCmp((byte*)& pjoin->info.devEUI, (byte*)& pwltab->DevEUI,8) == 0){
			rc=ndid;
			break; // we have a hit -> known DevEUI found
		}
		rc=-1;	// for this entry we have no hit, try next one
	}
	if (ndid == MAXNODES){	// reached end of table ?
		BHLOG(LOGLORAW) printf("  RegisterNode: Node not registered till WLTable-End\n");
		return(rc); // nothing more to do => DevEUI unknown -> have to reject this join request
	}
	if(pwltab->joined){		// already joined known node ?	-> was a rejoin ?
		BHLOG(LOGLORAW) printf("  RegisterNode: Node found in WLTable[%d] -> but already joined\n", ndid);
		// NDB[] already initialized with this node
		return(pwltab->nodeid-NODEIDBASE);	// return index to NDB[]
	}

	// Else, use same nodeid of WL-Table for NDB -> should be always a free entry in NodeDB
	pwltab->nodeid = NODEIDBASE + ndid;	// just to assure: base + index !
	pndb = & NDB[ndid];
	
	// now we can update WLTable & NDB for new joined node:
	BHLOG(LOGLORAW) printf("  RegisterNode: DevEUI registered -> update NodeDB[%i] with NodeID: 0x%02X\n",
			(unsigned char)ndid, (unsigned char)pwltab->nodeid );

	pwltab->joined = 1;				// mark WL Table entry as "joined"
	// initialize NDB entry:
	memcpy(&pndb->nodeinfo.devEUI, &pwltab->DevEUI,8);
	memcpy(&pndb->nodeinfo.joinEUI, &pwltab->AppEUI,8);
	pndb->nodeinfo.frmid[0] = 0;			// MSB: init "last frame msg id"
	pndb->nodeinfo.frmid[1] = 0;			// LSB:  -> First Session to be started with 0x01
	pndb->nodeinfo.vmajor	= pjoin->info.vmajor;	// get Version of BIoT protocol of node
	pndb->nodeinfo.vminor	= pjoin->info.vminor;
	pndb->nodecfg.channelidx= 0;					// we start with default Channel IDX = 0
	pndb->nodecfg.gwid		= pwltab->gwid;			// store predefined GW
	pndb->nodecfg.nodeid	= pwltab->nodeid;
	pndb->nodecfg.freqsensor= pwltab->reportfrq;	// [min] loop time of sensor status reports in Seconds
	pndb->nodecfg.verbose	= 0;					// reserved
	pndb->nodecfg.vmajor	= pjoin->info.vmajor;	// get Version of BIoT protocol of node
	pndb->nodecfg.vminor	= pjoin->info.vminor;	// gives room for backward support stepping
	pndb->nodecfg.nonce		= pndb->nodeinfo.frmid[1];	// init packet index by LSB of FrmID
	pndb->msg.ack			= 0;
	pndb->msg.retries		= 0;
	pndb->msg.idx			= 0;	// last msg idx => no msg received yet
	// FrmID & PkgID are handled/checked identical !

	pndb->msg.pkg	= (beeiotpkg_t*) NULL;	// preset Queue root point
	pndb->pwltab	= pwltab;	// back link to WL Table

	// T.b.d
	pndb->DevAddr = 0;
	pndb->AppSKey[0] = 0;
	pndb->NwSKey[0]  = 0;

    BHLOG(LOGLORAW) printf("  Node Joined ! Assigned: GWID:0x%02X, NodeID:0x%02X, ", pwltab->gwid, pwltab->nodeid);
	BHLOG(LOGLORAW) Printhex((byte*)& pndb->DevAddr, 4,  "DevAddr: 0x", 4); 
	BHLOG(LOGLORAW) printf("\n");
    BHLOG(LOGLORAW) Printhex( pndb->nodeinfo.devEUI,  8, "    DEVEUI: 0x-"); 
	BHLOG(LOGLORAW) printf("\n");
    BHLOG(LOGLORAW) Printhex( pndb->nodeinfo.joinEUI, 8, "   JOINEUI: 0x-"); 
	BHLOG(LOGLORAW) printf("\n");
    BHLOG(LOGLORAW) Printhex( pndb->AppSKey, 16,         "   AppSKEY: 0x-", 2);
	BHLOG(LOGLORAW) printf("\n");
    BHLOG(LOGLORAW) Printhex( pndb->NwSKey,  16,         "    NwSKEY: 0x-", 2);
	BHLOG(LOGLORAW) printf("\n");

	return(ndid);
}

//***************************************************************************
// JoinSrv_ValidateNode()
// Check Pkg-Header based on NDB Data
// 1. Check MIC based integrity
// 2. Search NDB for mutual NodeID
// 3. Check corresponding GW ID
// 4. Check WLTab for registerd but not joined Nodes
// Return:
// >=0	NDBidx of found entry
// -1	wrong GW ID
// -2	wrong NodeID
// -3	Known in WLTab but not joined -> no NDB entry
// -4   Known and Joined, but using wrong GWID -> should rejoin
//***************************************************************************
int JS_ValidatePkg(beeiotpkg_t* mystatus){
	int rc =0;
	int ndid;
	
	// 1. First create expected MIC and validate with Pkg-MIC
		// ToDo ... Also for JOIN requests ?

	// 2. check range of mutual GWID and NodeID (incl. JOIN requests !)
	if((mystatus->hd.sendID < NODEIDBASE) || (mystatus->hd.sendID > NODEIDBASE+MAXNODES)){
		// This is no known NodeID for this curr. Nw server instance !
		rc=-1;
		return(rc);
	}
	if((mystatus->hd.destID != GWIDx) && (mystatus->hd.destID != GWID1)){ 
		// This is no packet for this curr. Nw server instance !
		rc=-2;
		return(rc);
	}
	// 2. Compare Pkg Header with corresponding WLTab[] and NDB[] entries
	ndid = mystatus->hd.sendID - NODEIDBASE;	// extract NDB index
	if(!WLTab[ndid].joined){
		// This Node is known/registered but not joined yet ?
		if(mystatus->hd.sendID == NODEIDBASE && mystatus->hd.cmd == CMD_JOIN){
			// o.k. this is already a JOIN request -> lets do it...
			return(0);
		}
		// Known but not joined -> Node should initiate a JOIN request first -> rejected
		rc=-3;
		return(rc);
	}
	if(NDB[ndid].nodecfg.gwid != mystatus->hd.destID){
		// this joined node is (still) using the wrong GWID -> should rejoin
		BHLOG(LOGLORAW) printf("  JS_ValidateNode: joined node is (still) using the wrong GWID (0x%02X) -> should rejoin",
				mystatus->hd.destID);
		// Do it here or layer above ?
//		needaction = CMD_REJOIN;		// not implemented yet
//		BeeIoTFlow(needaction, mystatus, 0);
		rc=-4;
		return(rc);
	}
	
	// BIoT-Nw addresses assured now: "This was a package for us" -> now we could answer in any case

	// This check normally to be done by AppServer:
	// Check User Status data length
	if(mystatus->hd.frmlen > BIoT_FRAMELEN){
		BHLOG(LOGLORAW) printf("  BeeIoTParse: Wrong package size detected (%i) -> retry requested\n", 
					mystatus->hd.frmlen);		
		// for test purpose: dump payload in hex format
		BHLOG(LOGLORAR) hexdump((unsigned char*) mystatus, MAX_PAYLOAD_LENGTH);
		BHLOG(LOGLORAR) printf("\n");
		rc=-5;
		return(rc);
	}
	
	// Now we have a valid BeeIoT Package header: check the "cmd" for next BeeIoTWAN action
	// some debug output: assumed all is o.k.
		BHLOG(LOGLORAR) printf("  BeeIoTParse(0x%02X>0x%02X)", (unsigned char)mystatus->hd.sendID, (unsigned char)mystatus->hd.destID);
		BHLOG(LOGLORAR) printf("[%2i]:(cmd:%02d) ", (unsigned char) mystatus->hd.index, (unsigned char) mystatus->hd.cmd);
		BHLOG(LOGLORAR) Printhex((unsigned char*)mystatus->data, mystatus->hd.frmlen);

	return(ndid);	// everyting fine with this PKG -> forward to AppServer
}



// compare 2 binary streams with given length "binlen".
// rc=0 if equal: if not equal, rc provides # of equal bytes found in stream
int ByteStreamCmp(byte * bina, byte * binb, int binlen){
	int i;
	for (i=0; i<binlen; i++){
		if(bina[i]!= binb[i])
			return(i+1);
	}
	return(0);
}
