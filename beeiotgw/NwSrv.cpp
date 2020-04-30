/*******************************************************************************
* Sub Module: NwSrv.cpp
* The "NwSrv" Module is distributed under the New BSD license:
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
* NwSrv.cpp is released under the New BSD license. Go to the project
* home page for more info: https://github.com/beeiot
********************************************************************************
*/

#include <iostream>
#include <cstdint>
#include <cstring>
#include <sys/time.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>

#define BEEIOT_ACTSTRINGS
#include "BeeIoTWan.h"
#undef BEEIOT_ACTSTRINGS

#include "beeiot.h"
#include "gwqueue.h"	// using STL container classes
#include "regslora.h"
#include "beelora.h"
#include "radio.h"
#include "NwSrv.h"
#include "JoinSrv.h"
#include "BIoTApp.h"

extern unsigned int		lflags;		// BeeIoT log flag field

extern configuration	*cfgini;	// ptr. to struct with initial user params

// Global NodeDB[] for Node registration via JOIN command
// -> typeset see beelora.h; instance in JoinSrv.cpp
extern nodedb_t	NDB[];		// if 'NDB[x].nodeinfo.version == 0' => empty entry	


//******************************************************************************
// NwSrv() - Constructor
// INPUT:
//	ghwset		Array of LoRa Modem HW cfg settings
//  nmodem		# of Modem instances supp. in gwset[] (assumed: limited to MAXGW in main() )
// 
// RETURN:
//   0			all channel sessions finished successfully (never happens)
//  throw(EX_NWSRV_INIT)	Constructor failed
//
NwSrv::NwSrv(modemcfg_t *gwtab, int nmodem): gwhwset(gwtab){	

	// get current timestamp
	gettimeofday(&now, 0);
	strftime(TimeString, 80, "%d-%m-%y %H:%M:%S", localtime(&now.tv_sec));

	// check on valid input parameters
	if(!gwhwset || !nmodem){
		BHLOG(LOGBH) printf("NwS:%s  Start of BeeIoT-WAN Gateway Setup failed. wrong parameter (%p-%i)\n",
				TimeString, gwhwset, nmodem);
		throw(EX_NWSRV_INIT1);		// Gateway can not start
	}
	BHLOG(LOGBH) printf("NwS:%s  Start BeeIoT-WAN (v%d.%d) Gateway Setup\n",
			TimeString,  (int)BIoT_VMAJOR, (int)BIoT_VMINOR);

	int rc = wiringPiSPISetup(SPI0, SPIFRQ);	// initialize common SPI0 channel: MISO/MOSI/SCK
	if(rc == -1){
		BHLOG(LOGBH) printf("NwS: WiringPiSPISetup() failed: %s\n", errno);
		throw(EX_NWSRV_INIT2);		// Gateway can not start		
	}
	
	// Preset LoRa Modem Configuration	
	// Modem Setting for RX Mode: 
	int mid = 0;	// start with modem id = 0
	
	// Create LoRa modem objects
	for(mid; mid < nmodem; mid++){	// Min. 1 channel -> for JOIN needed
		try{
			gwhwset[mid].modem = new Radio(&gwhwset[mid]);		// instantiate LoRa Port 
		} catch (int excode){
				switch(excode){
				case EX_RADIO_INIT:
					printf("  NwS: Radio Exception (0x%04X) received\n", (unsigned int)excode); 
					gwhwset[mid].modem = (Radio *)NULL;	// no modem instance for this mid available
					if(mid==0){
						printf("  NwS: No LoRa Modem detected -> STOP BIoT Service\n"); 
					//	exit(0);	// no modem found at all -> give up
						throw(EX_NWSRV_INIT3);	// no modem found at all -> give up		

					}
					break;	// we have at least 1 modem, but stop scanning for more

				default: 
					printf("  NwS: Unknown exception received 0x%04X\n",(unsigned int)excode);
					std::exception();
					//	throw(EX_NWSRV_INIT3);	// no modem found at all -> give up		
					break;// ???
				}
		} // end of try()
	} // end of mid loop

	mactive = mid;	// remember max. # of active/instantiated  modems now served by NwS

	BHLOG(LOGLORAW) printf("NwS: LoRa Gateway Setup Done for %i modem\n", mactive);
} // end of NwSrv Constructor

//*****************************************************************************
// ~NwSrv() - Destructor
// Release of all instantiated (= active) modems
NwSrv::~NwSrv(void){
	for(int mid=0; mid < mactive; mid++){	// Min. 1 channel -> for JOIN needed
		delete gwhwset[mid].modem;	// remove LoRa Modem
		// create new MsgQueue and assign to new Modem session
	}		
}	// end of NwSrv Destructor


//******************************************************************************
// get number of activated and served modems by NwSrv
int	NwSrv::NwSrvModems(void){
	return(mactive);
}
	
//******************************************************************************
// NwNodeScan()
// Start each Modem in RXCont Mode -> waiting for LoRa Packages.
// Retrieved pkgs. by ISR are fetched from MsgQueue and forwarded to BIoTParse()
// Finally RXCont mode is entered again for affected modem.
// This routine serves endless searching for LoRa packages !
//
// INPUT:
//	None
// RETURN:
//   0			all channel sessions finished successfully (never happens)
//  throw(EX_NWSRV_INIT)	Constructor failed
//******************************************************************************
int NwSrv::NwNodeScan(void) {
			
	/* Working Example for PrintLoraStatus() Diagnostic Output
	LoRa Modem Register Status:     Version: 0x12  LoRa Modem OpMode : 0x80
	  MODEM_CONFIG1     : 0x72  DIO 0-3 Mapping   : 0x00  SYNC_WORD         : 0x12
	  MODEM_CONFIG2     : 0x74  DIO 4-5 Mapping   : 0x00  PREAMBLE_LENGTH   : 0x0008
	  MODEM_CONFIG3     : 0x04  LNA GAIN          : 0x23  PA_CONFIG         : 0x4F
	  PLL Bandwidth     : 0xD0  HOP_PERIOD        : 0xFF  PA_RAMP           : 0x09  
	  LoRa Modem Status : 0x10  FIFO_RX_BASE_ADDR : 0x00  IRQ_FLAGS_MASK    : 0x00
	  FIFO_RX_CURR_ADDR : 0x00  FIFO_TX_BASE_ADDR : 0x80  IRQ_FLAGS         : 0x00
	  FIFO_ADDR_PTR     : 0x00  FIFO_LAST_BYTE_WR : 0x00  RX_NB_BYTES       : 0x00
	  MAX_PAYLOAD_LENGTH: 0x80  Last PACKET_SNR   : 0x00  #of valid Headers : 0x0000
	  PAYLOAD_LENGTH    : 0x40  Last PACKET_RSSI  : 0x00  #of valid Packets : 0x0000
	  Current_RSSI      : 0x00  IRQ Level         : 0  
	*/

	gettimeofday(&now, 0);
	strftime(TimeString, 80, "%d-%m-%y %H:%M:%S", localtime(&now.tv_sec));
	BHLOG(LOGBH) printf("NwS:********************************************************************\n");
    BHLOG(LOGLORAW) printf("  NwS<%s>:  Start waiting for LoRa Node Packets in Contiguous Mode on all channels\n",
		TimeString);

  // run forever: wait for incoming packages via radio_irq_handler()
  while(1) {
	for(int mid=0; mid < mactive; mid++){	// ... for all discovered modems

		// activate modem to RXCont mode
		if((gwhwset[mid].modem->getopmode() & OPMODE_RX)!= OPMODE_RX){	
			if(mid==0){ // get new config only by status change of def. JOIN modem
				//re-read config.ini file again (could have been changed in between) 
				cfgini = getini((char*)CONFIGINI);
				if( cfgini == NULL){
					printf("    BeeIoT-Init: INI File not found at: %s\n", CONFIGINI);
					// continue with already buffered config struct data till we have access again
				}
				lflags	= (unsigned int) cfgini->biot_verbose;	// get the custom verbose mode again
				cfgini->loranumchn = mactive;	// limit # of supp. modems to what have been discovered
			}

			// Start LoRa Modem: in continuous read mode
			BHLOG(LOGLORAR) printf("  NwS: Enter RX_Cont Mode for Lora Modem%i\n", mid);
			gwhwset[mid].modem->startrx(RXMODE_SCAN, 0);	// RX in RX_CONT Mode (Beacon Mode)
			
			// ISR now waiting for rising DIO0 level -> starting receivepacket() directly
		}
		
		// Check RX Queue Status (len>0) and process package accordingly
		while(gwhwset[mid].gwq->MsgQueueSize() > 0){	// Do we have a package in the RX Queue ?

			gettimeofday(&now, 0);		// get current timestamp
			strftime(TimeString, 80, "%d-%m-%y %H:%M:%S", localtime(&now.tv_sec));

			int qlen = gwhwset[mid].gwq->MsgQueueSize();
			BHLOG(LOGLORAR) printf("  NwS<%s>: New RX Packet: Parsing MsgQueue[0] (Size:%i)\n", 
					TimeString, qlen);
			BHLOG(LOGLORAW) gwhwset[mid].gwq->PrintStatus();	// get MsgQueue STatus

			MsgBuffer * msg = &(gwhwset[mid].gwq->GetMsg());	// get ptr on queued MsgBuffer

			int rc = BeeIoTParse(msg);	// Start parsing payload data of msg	
			if( rc < 0){
				BHLOG(LOGLORAR) printf("  NwS: Parsing of MsgQueue[0] failed rc=%i\n", rc);
				// ToDo: Exit/Recover Code here ???
				// BHLOG(LOGLORAR) gwhwset[mid].gwq->PrintStatus();		
				// BHLOG(LOGLORAR) gwhwset[mid].modem->PrintLoraStatus(LOGALL);		
			}

			// Remove parsed package from Queue
			gwhwset[mid].gwq->PopMsg();
			BHLOG(LOGLORAW) gwhwset[mid].gwq->PrintStatus();

			BHLOG(LOGBH) printf("  NwSrv: LoraStatistic - Rcv:%u, Bad:%u, CRC:%u, O.K:%u, Fwd:%u\n",
				gwhwset[mid].cp_nb_rx_rcv, gwhwset[mid].cp_nb_rx_bad, gwhwset[mid].cp_nb_rx_crc, 
				gwhwset[mid].cp_nb_rx_ok, gwhwset[mid].cp_up_pkt_fwd);

			gettimeofday(&now, 0);
			strftime(TimeString, 80, "%d-%m-%y %H:%M:%S", localtime(&now.tv_sec));
			BHLOG(LOGBH) printf("\nNwS: %s: *************** Waiting for a new BIoTWAN package **************\n", TimeString);
		}
		
	} // end of Modem-Loop

	delay(NWSSCANDELAY);					// wait per loop in ms -> no time to loose.
  } // while(run NwService forever)

	// this point will never be reached !
	// end LoRa Gateway sessions
	for(int i=0; i< mactive; i++){
		delete gwhwset[i].gwq;		// release Modem Queue
		delete gwhwset[i].modem;	// and modem itself
	}
    return (0);
	
}





//*********************************************
// BeeIoTParse()
// Interprete received BeeIoT Status payload
// Input: 
//   message[]		stream of payload data (global)
//   receivedbytes	length of message stream
// Return: rc
//	0	All Data o.k. -> new CSV File entry created
//	1	All Data o.k. -> but duplicate to previous package -> no CSV File entry created
// -1	wrong NodeID
// -2   wrong GW ID
// -3   wrong package/status length	-> action: RETRY
// -4	wrong # of parsed status parameters -> action: RETRY
// -5	write to CSV file failed
// -6	CMD_GETSDLOG data package received, but not supported yet
// -7	JOIN request rejected
// -99   unknown CMD code -> ignored
//*********************************************
int NwSrv::BeeIoTParse(MsgBuffer * msg){
byte pkglen;
int rc=0; 
int ndid =0;
int idx=0;
int	needaction=0;
int loopid=0;
int nparam = 0;
char *ptr;					// ptr to next sensor parameter field
int	mid;
beeiotpkg_t mystatus;		// raw lora package buffer

	msg->getpkg(&mystatus);	// copy LoRa Pkg for parsing out of MsgBuffer
	mid = msg->getmid();
	// from now we take the payload as real data format:

	needaction = 0;
	pkglen =  (byte) mystatus.hd.frmlen + (byte)BIoT_HDRLEN + (byte)BIoT_MICLEN;

	// now check real data: MIC & Header first -> JOIN server
	rc = gwhwset[mid].jsrv->JS_ValidatePkg(&mystatus);
	if(rc < 0){
	// no valid Node packet header -> Header IDs unknown or Node not joined yet
		if (rc == -1 || rc == -2){ // -1: -> GWID;  -2: -> NodeID
			BHLOG(LOGLORAW) printf("  BeeIoTParse: Unknown Node/GW (NodeID:0x%02X -> GWID: 0x%02X) -> packet rejected\n", 
					(unsigned char)mystatus.hd.sendID, (unsigned char)mystatus.hd.destID);		
			// for test purpose: dump payload in hex format
			BHLOG(LOGLORAR) hexdump((unsigned char*) &mystatus, MAX_PAYLOAD_LENGTH);
			BHLOG(LOGLORAR) printf("\n");
			// ignore this package: no action
			gwhwset[mid].cp_nb_rx_bad++;	// bad pkg
			return(rc);
		}else if (rc == -3) {
			BHLOG(LOGLORAW) printf("  BeeIoTParse: Node registered but not joined yet (NodeID:0x%02X) -> Request a RE-JOIN\n", 
				(unsigned char)mystatus.hd.sendID);
			needaction = CMD_REJOIN;	// Request JOIN command from Node
			// but we return a positive ack
			BeeIoTFlow(needaction, &mystatus, mystatus.hd.sendID-NODEIDBASE, 0);
			rc=-3;
			gwhwset[mid].cp_nb_rx_ok++;		// incr. # of correct received packages	
			return(rc);
		}else if(rc == -4){
			BHLOG(LOGLORAW) printf("  BeeIoTParse: Node joined but not using assigned GWID 0x%02X -> should rejoin\n", 
				(unsigned char)mystatus.hd.destID);					
			gwhwset[mid].cp_nb_rx_ok++;		// incr. # of correct received packages	
			return(rc);			
		}else if (rc == -5){
			// wrong Framelen detected -> request RETRY
			BHLOG(LOGLORAW) printf("  BeeIoTParse: Wrong Framelen 0x%02X -> requesting a RETRY\n", 
				(unsigned char)mystatus.hd.frmlen);					
			BeeIoTFlow(CMD_RETRY, &mystatus, mystatus.hd.sendID-NODEIDBASE, 0);
			gwhwset[mid].cp_nb_rx_bad++;	// bad pkg
			return(rc);
		}
	}
	ndid = rc;	// we received Node-ID of curr. Pkg from JS
	// For (RE-)JOIN Request ndid is set to 0 by JS !

	gwhwset[mid].cp_nb_rx_ok++;			// incr. # of correct received packages	
	
	// Validate NwServer process flow:
	// - If we already know this BeeIoT Package: check the msgid
	if(ndid != 0){ // to be skipped during (RE-)JOIN request
		if(mystatus.hd.pkgid == gwhwset[mid].jsrv->NDB[ndid].msg.idx){	// do we have already received this pkgid from this node ?
			BHLOG(LOGLORAW) printf("  BeeIoTParse: package (%d) duplicated -> package dropped ->send ACK\n\n", (unsigned char) mystatus.hd.pkgid); // yes	
			// may be last ACK got lost, do it once again
			needaction = CMD_ACK;	// already known, but o.k.
			BeeIoTFlow(needaction, &mystatus, ndid, 0);
			// but we return a positive status
			rc=0;
			return(rc);
		}
	}	
	// is it a direct command for the NwServer ?
	switch (mystatus.hd.cmd){
	case CMD_NOP:// intentionally do nothing but ACK
		// intended to do nothing -> test message for communication check
		// for test purpose: dump payload in hex format
		BHLOG(LOGLORAW) printf("  BeeIoTParse: NOP -> Send ACK");	
		BHLOG(LOGLORAR) hexdump((unsigned char*) &mystatus, pkglen);
		needaction = CMD_ACK;	// we got it
		BeeIoTFlow(needaction, &mystatus, ndid, 0);
		BHLOG(LOGLORAW) printf("=> Done.\n");	
		rc= 0;	
		break;
	case CMD_JOIN:// Node to register for data collection
		gettimeofday(&now, 0);
		strftime(TimeString, 80, "%d-%m-%y %H:%M:%S", localtime(&now.tv_sec));
		BHLOG(LOGLORAW) printf("  BeeIoTParse: Node JOIN Requested, MsgID: 0x%02X, at %s\n",
				(unsigned char)mystatus.hd.pkgid, TimeString);
		BHLOG(LOGLORAR) hexdump((unsigned char*) &mystatus, pkglen);

		rc = gwhwset[mid].jsrv->JS_RegisterNode(&mystatus);	// evaluate by WLTable and create NDB[] entry
		if(rc >= 0){
			ndid = rc;	// get index of new NDB-entry 
			BHLOG(LOGLORAW) printf("  BeeIoTParse: New Node%i Send CONFIG (with new channel data) as ACK\n",ndid);
			// give node some time to recover from SendMsg before 
			delay(MSGRX1DELAY);
			needaction = CMD_CONFIG; // acknowledge JOIN request
			BeeIoTFlow(needaction, &mystatus, ndid, 0);
			BHLOG(LOGLORAW) printf("  => JOIN Done.\n");	
			rc= 0;	
		}else{
			BHLOG(LOGLORAW) printf("  BeeIoTParse: JOIN failed (rc=%i)\n", rc);
			rc=-7;
		}
		break;
	case CMD_REJOIN:	// Node was registered -> reactivate for data collection
		gettimeofday(&now, 0);
		strftime(TimeString, 80, "%d-%m-%y %H:%M:%S", localtime(&now.tv_sec));
		BHLOG(LOGLORAW) printf("  BeeIoTParse: Node REJOIN Requested, MsgID: 0x%02X, at %s\n", 
				(unsigned char) mystatus.hd.pkgid, TimeString);
		BHLOG(LOGLORAR) hexdump((unsigned char*) &mystatus, pkglen);

		rc = gwhwset[mid].jsrv->JS_RegisterNode(&mystatus);	// evaluate by WLTable and existing NDB[] entry
		if(rc >= 0){	// successfully reactivated
			ndid = rc;	// get idx of known NDB-entry 
			BHLOG(LOGLORAW) printf("  BeeIoTParse: Node Reactivated: Just Send CONFIG (with new channel data) as ACK\n");
			// give node some time to recover from SendMsg before 
			delay(MSGRX1DELAY);
			needaction = CMD_CONFIG; // acknowledge JOIN request
			BeeIoTFlow(needaction, &mystatus, ndid, 0);
			BHLOG(LOGLORAW) printf("  => REJOIN Done.\n");	
			rc= 0;	
		}else{
			BHLOG(LOGLORAW) printf("  BeeIoTParse: REJOIN failed (rc=%i)\n", rc);
			rc=-7;
		}
		break;
	case CMD_LOGSTATUS: // New BeeIoT node sensor data set received
		// First save dataset to BHDB
		BHLOG(LOGLORAW) printf("  BeeIoTParse: AppPkg -> Sensor-Status received\n");	

		// Forward Frame Payload to the assigned AppServer
		rc = gwhwset[mid].apps->AppProxy( (int) ndid, (char*) mystatus.data, (byte) mystatus.hd.frmlen, mid);

		// Was FramePayload complete ?
		if(rc == -1){   // wrong parameter set => request a resend of same message: RETRY
                    // lets acknowledge received package to sender to send it again
                    // ToDo: check for endless flow loop: only once
                    needaction = CMD_RETRY;
                    BeeIoTFlow(needaction, &mystatus, ndid, 0);  // header can be reused
                    rc= -4;
                    break;
		}else if(rc == -2){
                    // Sensor data could not be forwarded
                    rc= -5;
                    // ToDo: error recovery of CSV file
                    // AppProblem: no break here -> ACK pkg is needed to Node; Sent data was o.k.
		}

        BHLOG(LOGLORAW) printf("  BeeIoTParse: Send ACK\n");		
        needaction = CMD_ACK;	// no saved data, but o.k.
        BeeIoTFlow(needaction, &mystatus, ndid, 0);  // send ACK in sync mode

        if(rc == 1){
			BHLOG(LOGLORAW) printf("  BeeIoTParse: Processing RX1 Msg prepared by AppServer\n");		
			delay(MSGRX1DELAY);		// wait for RX1 window
			// ToDo: detect addon packages to send in RX1 if any
			//		E.g.	needaction = CMD_CONFIG;
			//			BeeIoTFlow(needaction, mystatus, 0);	// hand over mystatus for header data
		}else{
			BHLOG(LOGLORAW) printf("  BeeIoTParse: No RX1 Msg requested by AppServer\n");		
		}
		rc=0;	
		break;

	case CMD_GETSDLOG: // SD LOG data package received
		BHLOG(LOGLORAW) printf("  BeeIoTParse: SDLOG Save command: CMD(%d) not supported yet\n", (unsigned char) mystatus.hd.cmd);		
		// for test purpose: dump paload in hex format
		BHLOG(LOGLORAR) hexdump((unsigned char*) &mystatus, pkglen);

		// ToDO: we can send ACK or SD data packages directly ?!
		needaction = CMD_ACK;	// o.k. we got it
		BeeIoTFlow(needaction, &mystatus, ndid, 0);
		BHLOG(LOGLORAW) printf("  BeeIoTParse: ACK sent\n");		
		rc=-6; // but not supported yet
		break;
		
	default:	// unknown CMD for BEEIoT Protocol
		BHLOG(LOGLORAW) printf("  BeeIoTParse: unknown CMD(%d)\n", (unsigned char) mystatus.hd.cmd);		
		// for test purpose: dump payload in hex format
		BHLOG(LOGLORAR) hexdump((unsigned char*) &mystatus, pkglen);
//		needaction = 0;	// no
//		BeeIoTFlow(needaction, mystatus, 0);
		rc= -99;
		break;
	}

	return(rc);
} // end of BeeIoTParse()

//*****************************************************************************
// BeeIoTFlow():
// BIoT Flow Control action function supporting ACK/NOP/RETRY/CONFIG/REJOIN
// response packages. Package HD & data is created here.
// INPUT:
//	action	CMD code as defined in BIoT protocol (BIoTWAN.h)
//	pkg		typically the Lora package as sent by node -> used for response HD data
//	ndid	Node ID for NDB[] of receiving node
//	async	=0: function wait till TXDone Flag was set by ISR
//			=1: function returns immediately; caller has to poll TXDone flag if needed
// RETURN:
//	0	TX Done (if async=0) or TX initiated (if async =1)
// -1	unsupported CMD code
// -2	TX timeout occured
//
int NwSrv::BeeIoTFlow(u1_t action, beeiotpkg_t * pkg, int ndid, bool async){
beeiotpkg_t		actionpkg;	// new TX package buffer for creation
beeiot_header_t	*pack;	// ACK requires header only	-> reuse of pkg message field
beeiot_cfg_t	*pcfg;	// CONFIG has HD + CData	-> reuse of pkg message field
nodedb_t		*pndb;	// ptr to NDB[ndid]
byte			pkglen;	// length of raw TX pkg data (incl. MIC)
struct tm		*tval;	// values of current timestamp
int				mid=0;	// Modem ID (derived from GW id of pkg-header)
Radio *			Modem;	// Ptr on Modem used for transmission 

	BHLOG(LOGLORAR) printf("  BeeIoTFlow: New Action(async=%i): Send %s to Node:0x%02X\n", 
					async, beeiot_ActString[action], NODEIDBASE+(byte)ndid);
	pndb = &(gwhwset[0].jsrv->NDB[ndid]);	// get Cfg init data from NDB entry of this node

	switch (action){
	case CMD_NOP:	// fall thru by intention ==> like CMD_ACK
	case CMD_RETRY: // fall thru by intention ==> like CMD_ACK
	case CMD_ACK:	// by intention do nothing but ACK
		// give node some time to recover from SendMsg before 
		delay(RXACKGRACETIME);

		pack = (beeiot_header_t*) & actionpkg;
		// lets acknowledge action cmd related to received package to sender
		pack->destID= pkg->hd.sendID;	 // The BeeIoT node is the messenger
		pack->sendID= (u1_t) pndb->nodecfg.gwid; // New sender: its me
		pack->cmd	= action;			 // get our action command
		pack->pkgid = pkg->hd.pkgid;	 // get last pkg msgid
		pack->frmlen= 0;				 // send BeeIoT header for ACK only
		pkglen = BIoT_HDRLEN+BIoT_MICLEN;// just the BeeIoT header + MIC
		break;

	case CMD_REJOIN:	// Send  a simple REJOIN request (reuse ACK Format buffer)
		pack = (beeiot_header_t*) & actionpkg;
		// lets acknowledge action cmd related to received package to sender
		pack->destID = pkg->hd.sendID;	 // The BeeIoT node is the messenger
		pack->sendID = (u1_t) (GWIDx-cfgini->loradefchn);	// New sender: its me on Def.JOIN channel
		pack->cmd	 = action;			 // get our action command
		pack->pkgid  = pkg->hd.pkgid;	 // get last pkgid
		pack->frmlen = 0;				 // send BeeIoT header for ACK only
		pkglen = BIoT_HDRLEN+BIoT_MICLEN;// just the BeeIoT header + MIC
		break;

	case CMD_CONFIG: // Send runtime config params as assiged to node
		// give node some time to recover from SendMsg before 
		delay(RXACKGRACETIME);
		pcfg = (beeiot_cfg_t*) & actionpkg;	// use generic pkg space for CONFIG pkg

		// setup pkg header
		pcfg->hd.destID = pkg->hd.sendID;	// The BeeIoT node is the messenger
		pcfg->hd.sendID = (u1_t) (GWIDx-cfgini->loradefchn);	// New sender: its me on Def.JOIN channel
		pcfg->hd.cmd	= action;			// get our action command
		pcfg->hd.pkgid	= pkg->hd.pkgid;	// get last pkgid
		pcfg->hd.frmlen = sizeof(devcfg_t); // 

		pcfg->cfg.channelidx= pndb->nodecfg.channelidx;	// we start with assigned Channel IDX
		pcfg->cfg.gwid		= pndb->nodecfg.gwid;		// store predefined GW
		pcfg->cfg.nodeid	= pndb->nodecfg.nodeid;
		pcfg->cfg.freqsensor= pndb->nodecfg.freqsensor;	// [min] loop time of sensor status reports in Seconds
		pcfg->cfg.verbose	= pndb->nodecfg.verbose;	// reserved definition
		pcfg->cfg.vmajor	= pndb->nodecfg.vmajor;		// get Version of BIoT protocol of node
		pcfg->cfg.vminor	= pndb->nodecfg.vminor;		// gives room for backward support stepping		
		pcfg->cfg.nonce		= pndb->nodecfg.nonce;
		// get current local time for the client RTC update
		gettimeofday(&now, 0);
		tval = localtime(&now.tv_sec);
		pcfg->cfg.yearoff	= tval->tm_year-100;		// year offset since 2000
		pcfg->cfg.month		= tval->tm_mon+1;
		pcfg->cfg.day		= tval->tm_mday;
		pcfg->cfg.hour		= tval->tm_hour;
		pcfg->cfg.min		= tval->tm_min;
		pcfg->cfg.sec		= tval->tm_sec+3;			// sec. + CONFIG xfer correction
		
		pkglen = BIoT_HDRLEN + sizeof(devcfg_t) + BIoT_MICLEN;	// Cfg-Payload + BIOT Header
		BHLOG(LOGLORAR) hexdump((byte*) &actionpkg, pkglen);
	break;
	
	default: // don't know what to do  
		return(-1);	// unsupported CMD code
	}
	
	// Do final TX for all CMD cases: Select assigned modem handle
	mid = pndb->mid;					// get node assigned ModemID
	if(mid >= mactive){ 
		BHLOG(LOGLORAW) printf("\n  BIoTFlow: ################# Force mid:%i to limit-mid:%i ###################\n\n", mid, mactive-1);
		mid = 0;			// error recovery: limit modemid to JOIN default.
		pndb->mid = mid;	// safe state for next time
		// t.b.d.: there must be a wrong ID handling with NDB->mid
	}
	Modem = gwhwset[mid].modem;	// get corresponding modem object for TX
	
	BHLOG(LOGLORAR) printf("  BIoTFlow: TX[%i] GWID:0x%02X -> NodeID:0x%02X, CMD:0x%02X len:%i via Modem%i\n", 
			(int)actionpkg.hd.pkgid, (int)actionpkg.hd.sendID, (int)actionpkg.hd.destID, 
			(int)actionpkg.hd.cmd, (int)actionpkg.hd.frmlen, (int)mid);

	int rc = Modem->starttx((byte *)&actionpkg, pkglen, async );	// send LoRa pkg	

	if(rc<0){
		BHLOG(LOGLORAW) printf("  BIoTFlow: TX[%i] failed (RC%i) via Modem%i\n", (int)actionpkg.hd.pkgid, (int)rc, (int)mid); 
		return(-2);	// can be only TX TO
	}
	return(0);
}

