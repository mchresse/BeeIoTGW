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
NwSrv::NwSrv(gwbind_t &gwtab, int nmodem): gwt(gwtab){	
	gwhwset = gwt.gwset;

	// get current timestamp
	gettimeofday(&now, 0);
	strftime(TimeString, 80, "%d-%m-%y %H:%M:%S", localtime(&now.tv_sec));

	// check on valid input parameters
	if(!gwt.modem || !nmodem){	// if no modem setting list allocated or 0 gateways supported 
		BHLOG(LOGBH) printf("NwS:%s  Start of BeeIoT-WAN Gateway Setup failed. wrong parameter (%p-%i)\n",
				TimeString, gwt, nmodem);
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
			gwt.modem[mid] = new Radio(gwt, mid);  // instantiate LoRa Port[mid]
		} catch (int excode){
				switch(excode){
				case EX_RADIO_INIT:
					printf("  NwS: Radio Exception (0x%04X) received\n", (unsigned int)excode); 
					gwt.modem[mid] = (Radio *)NULL;	// no modem instance for this mid available
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

	// remember max. # of active/instantiated  modems now served by NwS (local & global)
	mactive = mid;
	gwt.nmodemsrv = mactive;

	BHLOG(LOGLORAW) printf("NwS: LoRa Gateway Setup Done for %i modem\n", mactive);
} // end of NwSrv Constructor

//*****************************************************************************
// ~NwSrv() - Destructor
// Release of all instantiated (= active) modems
NwSrv::~NwSrv(void){
	for(int mid=0; mid < mactive; mid++){
		delete gwt.modem[mid];	// remove LoRa Modem
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
//	-1			no active Modem available (mactive==0)
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
	// print status of default Join Modem (typ. the first one used)
	gwt.modem[gwt.joindef]->PrintModemStatus(LOGALL);
	gwt.modem[gwt.joindef]->PrintLoraStatus(LOGALL);
	
	gettimeofday(&now, 0);
	strftime(TimeString, 80, "%d-%m-%y %H:%M:%S", localtime(&now.tv_sec));
	BHLOG(LOGBH) printf("NwS:********************************************************************\n");
    BHLOG(LOGLORAW) printf("  NwS<%s>:  Start waiting for LoRa Node Packets in Contiguous Mode on all channels\n",
		TimeString);

  // run forever: wait for incoming packages via radio_irq_handler()
  int maxmod = mactive;	// get # of active modems only once
  int count =0;
  byte omstatus =0;

  while(1) { // start NwService loop (endless)
	for(int mid=0; mid < maxmod; mid++){// ... for all discovered modems
		count++;
		if(count%(10*10) == (10*10-1)){	// all 10 sec.
			printf("* ");
		}
		if(count%(2*600) == (2*600-1)){	// each 2. minute
			printf(" %i\n", count);
		}
		if(count >  ((cfgini->biot_loopwait+2)*600)){		// each loop + 2 minutes
			gwt.modem[mid]->PrintModemStatus(LOGALL);
			gwt.modem[mid]->PrintLoraStatus(LOGALL);
//			if(!gwt.modem[mid]->ChkLoraMode()){	// if no Lora Mode -> recover from FSK Mode
				gwt.modem[mid]->SetupRadio();	// FSK recovery needed: reset complete Modem
//			}
			count=0;
		}
		
		// Process NwS-Queue:
		// Check RX Queue Status (len>0) and process each package accordingly
		while(gwt.gwq->MsgQueueSize() > 0){	// Do we have a package in the RX Queue ?
			gettimeofday(&now, 0);		// get current timestamp
			strftime(TimeString, 80, "%d-%m-%y %H:%M:%S", localtime(&now.tv_sec));

			int qlen = gwt.gwq->MsgQueueSize();
			BHLOG(LOGLORAR) printf("  NwS<%s>: New RX Packet: Parsing MsgQueue[0] (Size:%i)\n", 
					TimeString, qlen);
			BHLOG(LOGLORAW) gwt.gwq->PrintStatus();	// get MsgQueue STatus

			MsgBuffer * msg = &(gwt.gwq->GetMsg());	// get ptr on queued MsgBuffer

			int rc = BeeIoTParse(msg);	// Start parsing payload data of msg	
			if( rc < 0){
				BHLOG(LOGLORAR) printf("  NwS: Parsing of MsgQueue[0] failed rc=%i\n", rc);
				// ToDo: Exit/Recover Code here ???
				// BHLOG(LOGLORAR) gwt.gwq->PrintStatus();		
				// BHLOG(LOGLORAR) gwt.modem[mid]->PrintLoraStatus(LOGALL);		
			}

			// Remove parsed package from Queue
			gwt.gwq->PopMsg();
			BHLOG(LOGLORAW) gwt.gwq->PrintStatus();

			BHLOG(LOGBH) printf("  NwSrv: LoraStatistic - Rcv:%u, Bad:%u, CRC:%u, O.K:%u, Fwd:%u\n",
				gwt.cp_nb_rx_rcv, gwt.cp_nb_rx_bad, gwt.cp_nb_rx_crc, 
				gwt.cp_nb_rx_ok, gwt.cp_up_pkt_fwd);
			count =0;
		}
		
		// activate modem to RXCont mode
		omstatus = gwt.modem[mid]->getopmode();
		if((omstatus & OPMODE_RX)!= OPMODE_RX){	
			if(mid==0){ // get new config only by status change of def. JOIN modem
				//re-read config.ini file again (could have been changed in between) 
				configuration* pcfg;
				pcfg = getini((char*)CONFIGINI);
				if( pcfg == NULL){ // config file existing and parsed ?
					printf("    BeeIoT-NwS: INI File not found at: %s\n", CONFIGINI);
					if( cfgini == NULL){ 
						// This should not happen: Cfgini should have been initiaized in initall() once!!!
						printf("    BeeIoT-NwS: INI FIle not found at: %s\n", CONFIGINI);
						exit(EXIT_FAILURE);	// so we have to give up: heavy FS problems...?!
					}
					// continue with already buffered cfgini struct data till we have access again
				}else{
					cfgini = pcfg;	// we have a new config parameter set
					lflags	= (unsigned int) cfgini->biot_verbose;	// get the custom verbose mode again
					cfgini->loranumchn = mactive;	// limit # of supp. modems to what have been discovered
				} // (new) valid cfg-data available
			}
			count = 0;

			// Start LoRa Modem: in continuous read mode again
			BHLOG(LOGLORAR) printf("  NwS: Enter RX_Cont Mode for Lora Modem%i (omstatus=%02X)\n", mid, (unsigned char)omstatus);
			int rc = gwt.modem[mid]->startrx(RXMODE_SCAN, 0);	// RX in RX_CONT Mode
			if(rc<0){
				gwt.modem[mid]->SetupRadio();	// FSK recovery needed: reset complete Modem
				gwt.modem[mid]->startrx(RXMODE_SCAN, 0);	// retry RX CONT Mode
			}
			delay(500);		// wait some time (500ms) till status gets active
			// ISR is now waiting for DIO0 port change
			gettimeofday(&now, 0);
			strftime(TimeString, 80, "%d-%m-%y %H:%M:%S", localtime(&now.tv_sec));
			BHLOG(LOGBH) printf("\nNwS: %s: *************** Waiting for a new BIoTWAN package **************\n", TimeString);
		}
	} // end of Modem-Loop
	if(!mactive){
		gettimeofday(&now, 0);
		strftime(TimeString, 80, "%d-%m-%y %H:%M:%S", localtime(&now.tv_sec));
		printf("\nNwS: %s: No active LoRa Modem left -> Stop Server Scan\n", TimeString);
		return(-1);
	}

	delay(NWSSCANDELAY);					// wait per loop in ms -> no time to loose.
  } // while(run NwService forever)

	// this point will never be reached !
	// end LoRa Gateway sessions

	return(0);	
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
ackbcn_t * pbcn;			// ptr on Beacon ack. frame

	msg->getpkg(&mystatus);	// copy LoRa Pkg for parsing out of MsgBuffer
	mid = msg->getmid();

	// from now we take the payload as real data format:

	needaction = 0;
	pkglen =  (byte) mystatus.hd.frmlen + (byte)BIoT_HDRLEN + (byte)BIoT_MICLEN;

	// now check real data: MIC & Header first -> JOIN server
	rc = gwt.jsrv->JS_ValidatePkg(&mystatus);
	if(rc < 0){
	// no valid Node packet header -> Header IDs unknown or Node not joined yet
		if (rc == -1 || rc == -2){ // -1: -> GWID;  -2: -> NodeID
			BHLOG(LOGLORAW) printf("  BeeIoTParse: Unknown Node/GW (NodeID:0x%02X -> GWID: 0x%02X) -> packet rejected\n", 
					(unsigned char)mystatus.hd.sendID, (unsigned char)mystatus.hd.destID);		
			// for test purpose: dump payload in hex format
			BHLOG(LOGLORAR) hexdump((unsigned char*) &mystatus, MAX_PAYLOAD_LENGTH);
			BHLOG(LOGLORAR) printf("\n");
			// ignore this package: no action
			gwt.cp_nb_rx_bad++;	// bad pkg
			return(rc);
		}else if (rc == -3) {
			BHLOG(LOGLORAW) printf("  BeeIoTParse: Node registered but not joined yet (NodeID:0x%02X) -> Request a RE-JOIN\n", 
				(unsigned char)mystatus.hd.sendID);
			needaction = CMD_REJOIN;	// Request JOIN command from Node
			// but we return a positive ack
			BeeIoTFlow(needaction, &mystatus, mystatus.hd.sendID-NODEIDBASE, 0);
			rc=-3;
			gwt.cp_nb_rx_ok++;		// incr. # of correct received packages	
			return(rc);
		}else if(rc == -4){
			BHLOG(LOGLORAW) printf("  BeeIoTParse: Node joined but not using assigned GWID 0x%02X -> should rejoin\n", 
				(unsigned char)mystatus.hd.destID);					
			gwt.cp_nb_rx_ok++;		// incr. # of correct received packages	
			return(rc);			
		}else if (rc == -5 || rc == -6){
			// wrong Framelen detected -> request RETRY
			BHLOG(LOGLORAW) printf("  BeeIoTParse: Wrong Framelen 0x%02X or payload corrupted (rc:%i)-> requesting a RETRY\n",
				(unsigned char)mystatus.hd.frmlen, rc);
			BeeIoTFlow(CMD_RETRY, &mystatus, mystatus.hd.sendID-NODEIDBASE, 0);
			gwt.cp_nb_rx_bad++;	// bad pkg
			return(rc);
		}
	}
	ndid = rc;	// we received Node-ID of curr. Pkg from JS
	// For (RE-)JOIN Request ndid is set to 0 by JS !

	gwt.cp_nb_rx_ok++;			// incr. # of correct received packages	
	
	// Validate NwServer process flow:
	// - update MsgBuffer-HD by pkgid (could not be done by ISR yet)
	msg->setpkgid_ack(mystatus.hd.pkgid,0);	
	// - If we already know this BeeIoT Package: check the msgid
	if(ndid != 0){ // to be skipped during (RE-)JOIN request (ndid==0)
		if(mystatus.hd.pkgid == gwt.jsrv->NDB[ndid].msg.pkgid){	// do we have already received this pkgid from this node ?
			BHLOG(LOGLORAW) printf("  BeeIoTParse: package (%d) duplicated -> package dropped ->send ACK\n\n", (unsigned char) mystatus.hd.pkgid); // yes	
			// may be last ACK got lost, do it once again
			needaction = CMD_ACK;	// already known, but o.k.
			BeeIoTFlow(needaction, &mystatus, ndid, 0);
			// but we return a positive status
			rc=0;
			return(rc);
		}
		gwt.jsrv->NDB[ndid].msg.mid = mid;	// safe mid for any later action
	}else{ // for JOIN always NDBID=0 is used.
		gwt.jsrv->NDB[0].msg.mid = gwt.jsrv->NDB[0].middef; // use JOIN default modem ID as mid
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

		rc = gwt.jsrv->JS_RegisterNode(&mystatus);	// evaluate by WLTable and create NDB[] entry
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

		rc = gwt.jsrv->JS_RegisterNode(&mystatus);	// evaluate by WLTable and existing NDB[] entry
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
		rc = gwt.apps->AppProxy( (int) ndid, (char*) mystatus.data, (byte) mystatus.hd.frmlen, mid);

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
		
	case CMD_BEACON: // BeeIoT node beacon received
		BHLOG(LOGLORAW) printf("  BeeIoTParse: AppPkg -> Beacon received\n");	

		// No Forward Frame Payload to the assigned AppServer by now
		// rc = gwt..apps->AppProxy( (int) ndid, (char*) mystatus.data, (byte) mystatus.hd.frmlen, mid);
		
		rc = NwSBeacon(mid, ndid, msg);
		
        BHLOG(LOGLORAW) printf("  BeeIoTParse: Send ACK\n");		
        needaction = CMD_ACKBCN;	// no saved data, but o.k.
		// predefine beacon ack data to pkg frame (harder to achieve in generic beeiotflow()
		pbcn = (ackbcn_t*) &mystatus.data;	// get start of payload frame
		pbcn->rssi = msg->getrssi();
		pbcn->snr  = msg->getsnr();
        BeeIoTFlow(needaction, &mystatus, ndid, 0);  // send ACK in sync mode
		
		rc=0;	
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
//			For AckBeacon: pkg data frame is already prepared with response data (ackbcn_t)
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
	pndb = &(gwt.jsrv->NDB[ndid]);	// get Cfg init data from NDB entry of this node

	switch (action){
	case CMD_NOP:	// fall thru by intention ==> like CMD_ACK
	case CMD_RETRY: // fall thru by intention ==> like CMD_ACK
	case CMD_ACK:	// by intention do nothing but ACK
	case CMD_ACKBCN:// Beacon ACK
		
		// give node some time to recover from SendMsg before 
		delay(RXACKGRACETIME);

		pack = (beeiot_header_t*) & actionpkg;
		// lets acknowledge action cmd related to received package to sender
		pack->destID= pkg->hd.sendID;	 // The BeeIoT node is the messenger
		pack->sendID= (u1_t) pndb->nodecfg.gwid; // New sender: its me
		pack->cmd	= action;			 // get our action command
		pack->pkgid = pkg->hd.pkgid;	 // get last pkg msgid
		pack->res	= 0;
		if(action == CMD_ACKBCN){
			// assumed pbcn data is already defined by caller: e.g. rssi & snr
			memcpy(&actionpkg.data, &pkg->data, sizeof(ackbcn_t));
			// ToDO: add MIC field at end of actionpkg.data
			pack->frmlen= sizeof(ackbcn_t);	// length of ackbcn data only
			pkglen = BIoT_HDRLEN + pack->frmlen + BIoT_MICLEN;// just the BeeIoT header + MIC			
			BHLOG(LOGLORAR) hexdump((byte*) &actionpkg, pkglen);
		}else{
			pack->frmlen= 0;				 // send BeeIoT header for ACK only
			pkglen = BIoT_HDRLEN+BIoT_MICLEN;// just the BeeIoT header + MIC
			// ToDO: add MIC field at end of actionpkg.data
		}
		break;

	case CMD_REJOIN:	// Send  a simple REJOIN request (reuse ACK Format buffer)
		pack = (beeiot_header_t*) & actionpkg;
		// lets acknowledge action cmd related to received package to sender
		pack->destID = pkg->hd.sendID;	 // The BeeIoT node is the messenger
		pack->sendID = (u1_t) (GWIDx-gwt.joindef);	// New sender: its me on Def.JOIN channel
		pack->cmd	 = action;			 // get our action command
		pack->pkgid  = pkg->hd.pkgid;	 // get last pkgid
		pack->frmlen = 0;				 // send BeeIoT header for ACK only
		pack->res	= 0;
		pkglen = BIoT_HDRLEN+BIoT_MICLEN;// just the BeeIoT header + MIC
		// ToDO: add MIC field to end of actionpkg.data
		break;

	case CMD_CONFIG: // Send runtime config params as assiged to node
		// give node some time to recover from SendMsg before 
		delay(RXACKGRACETIME);
		pcfg = (beeiot_cfg_t*) & actionpkg;	// use generic pkg space for CONFIG pkg

		// setup pkg header
		pcfg->hd.destID = pkg->hd.sendID;	// The BeeIoT node is the messenger
		pcfg->hd.sendID = (u1_t) (GWIDx-gwt.joindef);	// New sender: its me on Def.JOIN channel
		pcfg->hd.cmd	= action;			// get our action command
		pcfg->hd.pkgid	= pkg->hd.pkgid;	// get last pkgid
		pcfg->hd.frmlen = sizeof(devcfg_t); // 
		pcfg->hd.res	= 0;

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

		// ToDO: add MIC field to end of actionpkg.data
		break;
	
	default: // don't know what to do  
		return(-1);	// unsupported CMD code
	}
	
	// Do final TX for all CMD cases: Select assigned modem handle
	mid = pndb->msg.mid;		// get node pkg used  ModemID
	if(mid >= mactive){			// if mid is out of active modem scope (should never happen!)
		BHLOG(LOGLORAW) printf("\n  BIoTFlow: ################# Force mid:%i to default-mid:%i ###################\n\n", mid, pndb->middef);
		mid = pndb->middef;		// error recovery: limit modemid to assigned JOIN default.
		pndb->msg.mid = mid;	// safe state for next time also
		// t.b.d.: there must be a wrong ID handling with NDB->mid ?
	}
	
	BHLOG(LOGLORAR) printf("  BIoTFlow: TX[%i] GWID:0x%02X -> NodeID:0x%02X, CMD:0x%02X len:%i via Modem%i\n", 
			(int)actionpkg.hd.pkgid, (unsigned char)actionpkg.hd.sendID, (unsigned char)actionpkg.hd.destID, 
			(unsigned char)actionpkg.hd.cmd, (int)actionpkg.hd.frmlen, (int)mid);

	Modem = gwt.modem[mid];		// get corresponding modem object for TX
	int rc = Modem->starttx((byte *)&actionpkg, pkglen, async );	// send LoRa pkg	

	if(rc<0){
		BHLOG(LOGLORAW) printf("  BIoTFlow: TX[%i] failed (RC%i) via Modem%i\n", (int)actionpkg.hd.pkgid, (int)rc, (int)mid); 
		return(-2);	// can be only TX TO
	}
	return(0);
}

//*****************************************************************************
// NwSBeacon():
// Process BIoT Beacon package: parse payload and print data 
// INPUT:
//  mid		Modem ID of receiving modem 
//	ndid	Node ID for NDB[] of sender node
//	msg		Beacon MsgBuffer sent by node: need SNR & RSSI
// RETURN:
//	0	beacon processed
// -1	wrong beacon format (crc1 wrong)
// -2	wrong beacon format (crc2 wrong)
//
int NwSrv::NwSBeacon(u1_t mid, int ndid, MsgBuffer *msg){
	beeiotpkg_t pbuf;					// copy buffer of a pkg frame
	msg->getpkg((beeiotpkg_t*) &pbuf);	// cast std. pkg format on beacon format
	beeiot_beacon_t * pbcn = (beeiot_beacon_t *) & pbuf;// ptr on a beacon frame
	beacon_t * bcn = (beacon_t *) & pbuf.data;			// ptr on Data frame of beacon
	union {
        float f;
        int i;
    } f;
	float latd, lond;
	uint8_t * buffer;

	// ESP32-Arduino is little endian; Raspberry-Debian is big endian !
	// correct float values to big endian format
	buffer = bcn->lat;	
    f.i = buffer[0] |  (buffer[1] <<8) |  (buffer[2] <<16) |  (buffer[3] <<24);
	latd = f.f;
	buffer = bcn->lon;	
    f.i = buffer[0] |  (buffer[1] <<8) |  (buffer[2] <<16) |  (buffer[3] <<24);
	lond = f.f;

	// by now just print beacon data to diag port + no crc check
	BHLOG(LOGLORAW) printf("  BIoT_Beacon[%i] %02i:%02i:%02i - ", (int)pbcn->hd.pkgid, (int)bcn->hour, (int)bcn->min, (int)bcn->sec);
	BHLOG(LOGLORAW) printf("MID:%i, Node:0x%02X, RSSI:%i, SNR:%i,  ", (int)mid, (unsigned char)pbcn->hd.sendID, msg->getrssi(), msg->getsnr());	
	BHLOG(LOGLORAW) printf("GPS[%f,%f,%im])  Info:%i", latd, lond, (int)bcn->alt, (int)bcn->info);
	BHLOG(LOGLORAW) printf("  (CRC1:%i, CRC2:%i)\n", (int)bcn->crc1, (int)bcn->crc2);

	last_beaconid = (int)msg->getpkgid();

	return(0);
}
