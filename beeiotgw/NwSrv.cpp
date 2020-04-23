/*******************************************************************************
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
/*******************************************************************************
 * Sub Module NwSrv.cpp
 *
 *******************************************************************************
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include <sys/time.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>


#include "regslora.h"

#define BEEIOT_ACTSTRINGS
#include "BeeIoTWan.h"
#undef BEEIOT_ACTSTRINGS

#include "beelora.h"
#include "beeiot.h"
#include "gwqueue.h"
#include "radio.h"


extern unsigned int	lflags;               // BeeIoT log flag field

/*******************************************************************************
 *
 * BIoTWAN LoRA protocol Constants: Please customize for your new network:
 * (see also settings in beelora.h)
 *
 *******************************************************************************/

beeiotpkg_t MyRXData[MAXRXPKG];     // RX-Pkg Queue: received messages for userland processing (decoded)
// RX Queue buffer & control
byte    BeeIotRXFlag;            // Semaphore for received message(s) 0 ... MAXRXPKG-1
byte    BeeIotTXFlag;            // Semaphore for a single sent message
byte    RXPkgIsrIdx ;            // WR index on next RX Queue Package for ISR callback Write
byte    RXPkgSrvIdx ;            // RD index on next RX Queue Package for BIoTParse() Service



//******************************************************************************
// Global NodeDB[] for Node registration via JOIN command
#define SCANDELAY	1000		// in main(): wait loop delay in ms

// Central Database of all measured values and runtime parameters
extern dataset			bhdb;       // central beeIoT data DB
extern configuration * cfgini;			// ptr. to struct with initial parameters
extern modemcfg_t	*	gwset;		// GateWay related config sets for Radio Instantiation

extern nodedb_t NDB[];			// if 'NDB[x].nodeinfo.version == 0' => empty entry
// -> typeset see beelora.h; instance in JoinSrv.cpp


static const int SPI0	= 0;        // use RPi SPI channel 0
static const int SPIFRQ = 500000;   // use SPI channel Frequency 0,5MHz

struct timeval now;          // current tstamp used each time a time check is done
extern unsigned long txstart;       // tstamp when TX Mode was entered
extern unsigned long txend;         // Delta: now - txstart
extern unsigned long rxtime;        // tstamp when last rx package arrived
char	TimeString[128];            // contains formatted Timestamp string of each loop(in main())

// Initialize BIoT WAN default addresses for joining nodes
byte	gwid0	= GWIDx;            // set default ID JOIN GW Instance
byte	gwid1	= GWID1;            // set ID of transmission GW Instance
byte	nodeid	= NODEIDBASE;       // get curr. ID of nodeID we are talking with

extern uint32_t cp_nb_rx_ok;        // # of correct received packages (correct header) -> Statistics for BIoTApp() calls
extern uint32_t cp_nb_rx_rcv;       // # of raw received LoRa packages
	
//******************************************************************************
// local Function prototypes
int  NwNodeScan		(modemcfg_t *gwhwset, int nmodem, int defchannel);
int  BeeIoTParse	(beeiotpkg_t * mystatus);
void BeeIoTFlow		(u1_t action, beeiotpkg_t * pkg, int ndid, int async);

// in JoinSrv.cpp:
extern int	JS_RegisterNode	(beeiotpkg_t * joinpkg, int async);
extern int	JS_ValidatePkg	(beeiotpkg_t* mystatus);
extern int	ByteStreamCmp	(byte * bina, byte * binb, int binlen);
extern int	JS_AppProxy	(int ndid, char * framedata, byte framelen);


/******************************************************************************
 * NwNodeScan()
 * INPUT:
 *	ghwset		Array of LoRa Modem HW cfg settings
 *  nmodem		# of Modem instances in gwset[] (assumed: limited to MAXGW in main() )
 *  defchannel	Default Channel cfg set for jOIN requests
 * RETURN:
 *   0			all channel session finshed successfully (but sessions are endless)
 *  -1			wrong input params
 ******************************************************************************/
int NwNodeScan (modemcfg_t *gwhwset, int nmodem, int defchannel) {
    uint32_t lasttime;
	int rc;

	if(!gwhwset || !nmodem)	// check input
		return(-1);
	
	printf("  NwS: BeeIoT-WAN NwServer: Start Node Scan\n");

	// get current timestamp
	gettimeofday(&now, 0);
	strftime(TimeString, 80, "%d-%m-%y %H:%M:%S", localtime(&now.tv_sec));
	 BHLOG(LOGBH) printf("  NwS<%s>: Setup started for BIoTWAN v%d.%d\n", 
		TimeString,  (int)BIoT_VMAJOR, (int)BIoT_VMINOR);

	wiringPiSPISetup(SPI0, SPIFRQ);	// initialize common SPI0 channel: MISO/MOSI/SCK
	 
	// Init NodeDB[] for new registrations:
	for(int i=0; i<MAXNODES; i++){
		nodedb_t * pndb = &NDB[i];
//		BHLOG(LOGBH) printf("  NwS: NDB:%p - NDB[%i]:%p - pndb:%p\n", &NDB, i, &NDB[i], pndb);
		NDB[i].nodeinfo.vmajor =0; // free entry if 'NDB[x].nodeinfo.vmajor == 0'
		NDB[i].nodeinfo.vminor =0;
		NDB[i].nodeinfo.devEUI[0] =0;
		NDB[i].nodeinfo.devEUI[1] =0;
		NDB[i].nodeinfo.devEUI[2] =0;
		NDB[i].nodeinfo.devEUI[3] =0;
		pndb->nodecfg.gwid		= gwid0;
		pndb->nodecfg.nodeid	= NODEIDBASE + i;	// a bit critical here: NODEIDx is defined statically
		pndb->nodecfg.vmajor	= BIoT_VMAJOR;	// Major + Minor version: V01.00
		pndb->nodecfg.vminor	= BIoT_VMINOR;
		pndb->nodecfg.verbose	= 0;
		pndb->nodecfg.channelidx =0;
		pndb->nodecfg.freqsensor = (int) 60;	// [min] reporting frequence of status pkg.
		NDB[i].msg.ack = 0;
		NDB[i].msg.idx = 0;
		NDB[i].msg.retries = 0;
		NDB[i].msg.rssi = 0;
		NDB[i].msg.snr = 0;	
		NDB[i].msg.pkg = (beeiotpkg_t*)NULL;
	}
	
	// Initialize BIoT WAN default addresses for joining nodes
	// For Node session communication, NDB[] values have to be used (as init by WLTab).
	gwid0	= GWIDx;
	gwid1	= GWID1;
	nodeid	= NODEIDBASE;

	// Preset RX QUEUE: MyRXData[MAXRXPKG]
	BeeIotRXFlag = 0;
	BeeIotTXFlag = 0;
	RXPkgSrvIdx  = 0;
	RXPkgIsrIdx	 = 0;

	// Preset LoRa Modem Configuration	
	// Modem Setting for RX Mode: 

// for Queue testing
	//	MsgQueue<MsgBuffer> * gwq = init_gwqueue(0);		// create
/*	MsgQueue gwq; // create
	gwq.PrintStatus();
	MsgBuffer mb(99,2,0,0);
	gwq.PushMsg(mb);	// MsgBuffer is moved to Queue (!)
	gwq.PrintStatus();
	gwq.PopMsg();		// MsgBuffer is deleted !
	gwq.PrintStatus();
*/	// end of testing
	
	// 1. Create LoRa modem object
	// 2. Create MSG Queue
	// 3. Assign MsgQueue to LoRa Modem
	for(int mid=0; mid < nmodem; mid++){	// Min. 1 channel -> for JOIN needed
		gwhwset[mid].modem = new Radio(&gwhwset[mid]);		// instantiate LoRa Port 
		// create new MsgQueue and assign to new Modem session
		gwset[mid].gwq = new MsgQueue;		// create & store global Msg Queue ptr
		gwhwset[mid].modem->assign_gwqueue(*gwset[mid].gwq); // assign Queue to Modem
		gwset[mid].gwq->PrintStatus();
	}
	
	BHLOG(LOGLORAW) printf("  NwS: LoRa Modem Setup finished; Start Channel scanning...\n");
	
	// Activate modemwise in order
	for(int mid=0; mid < nmodem; mid++){	// Min. 1 channel -> for JOIN needed
		// get current timestamp
		gettimeofday(&now, 0);
		strftime(TimeString, 80, "%d-%m-%y %H:%M:%S", localtime(&now.tv_sec));

		if(gwhwset[mid].modem){
			BHLOG(LOGLORAR) printf("  NwS: Lora%i Register Configuration:\n", (int)gwhwset[mid].modemid);
			BHLOG(LOGLORAR)gwhwset[mid].modem->PrintLoraStatus(LOGALL);
			// Start LoRa Read Loop in  "continuous read" Mode
			gwhwset[mid].modem->startrx(RXMODE_SCAN, 0);	// RX in RX_CONT Mode 
		}else{
			if (mid == 0){	// was it the first LoRa modem which failed ?
				return(-2);	// we need minimum 1 Modem -> break program.
			}else{ // no 2. modem detected
				--nmodem;
				BHLOG(LOGLORAR) printf("NwS: FallBack Multi -> Single CHannel Mode");
				break;
			}
		}			
	}
		
/* Working Example Output right from here:
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

	BHLOG(LOGBH) printf("NwS:********************************************************************\n");
    BHLOG(LOGLORAW) printf("  NwS<%s>:  Start waiting for LoRa Node Packets in Contiguous Mode by all channels\n",
		TimeString);

// run forever: wait for incoming packages via radio_irq_handler()
while(1) {
	for(int mid=0; mid < nmodem; mid++){
		// ISR waiting for rising DIO0 level -> starting receivepacket() directly
		while(BeeIotRXFlag){	// Do we have a package in the RX Queue ?
			// check RX Queue BeeIoT WAN Status and process package accordingly
			// get current timestamp
			gettimeofday(&now, 0);
			strftime(TimeString, 80, "%d-%m-%y %H:%M:%S", localtime(&now.tv_sec));
			BHLOG(LOGLORAR) printf("  NwS<%s>: New RX Packet: Parsing MsgQueue[0] (Size:%i)\n", 
					TimeString, (int)BeeIotRXFlag);
			rc = BeeIoTParse(&MyRXData[RXPkgSrvIdx]);
			if(rc < 0){
				BHLOG(LOGLORAR) printf("  NwS: Parsing of MsgQueue[0] failed rc=%i\n", rc);
				// ToDo: Exit/Recover Code here ???
				// BHLOG(LOGLORAR) gwset[mid].gwq->PrintStatus();		
				// BHLOG(LOGLORAR) gwhwset[mid].modem->PrintLoraStatus(LOGALL);		
			}

			// switch to next Queue element
			if(++RXPkgSrvIdx >= MAXRXPKG){  // RX Queue end reached ?
			  BHLOG(LOGLORAR) printf("  NwS: RX Buffer end reached, switch back to buffer[0]\n");
			  RXPkgSrvIdx=0;  // wrap around
			}
			BeeIotRXFlag--;   // however,  we can release one more RX Queue buffer
	
			BHLOG(LOGLORAR) printf("  NwS: New RX-Queue Status: SrvIdx:%i, IsrIdx:%i, RXFlag:%i\n",
				(int)RXPkgSrvIdx, (int)RXPkgIsrIdx, (int)BeeIotRXFlag);
		}
		
		if((gwhwset[mid].modem->getopmode() & OPMODE_RX)!= OPMODE_RX){

			if(mid==0){ // get new config only by status change of def. JOIN modem
				//re-read config.ini file again (could have been changed in between) 
				cfgini = getini((char*)CONFIGINI);
				if( cfgini == NULL){
					printf("    BeeIoT-Init: INI File not found at: %s\n", CONFIGINI);
					// exit(EXIT_FAILURE);
					// continue with already buffered config struct data till we have access again
				}
				lflags	= (unsigned int) cfgini->biot_verbose;	// get the custom verbose mode again
				cfgini->loranumchn = nmodem;	// limit # of supp. modems to what have been told by main()
			}

			// Start LoRa Mode: continuous read loop - again
			BHLOG(LOGLORAR) printf("  NwS: Enter RX_Cont Mode for Lora Modem%i\n", mid);
			gwhwset[mid].modem->startrx(RXMODE_SCAN, 0);	// RX in RX_CONT Mode (Beacon Mode)
			gettimeofday(&now, 0);
			strftime(TimeString, 80, "%d-%m-%y %H:%M:%S", localtime(&now.tv_sec));
			BHLOG(LOGBH) printf("NwS: %s: ********* Wait for New Packet ********\n", TimeString);
			//	gwhwset[mid].modem->PrintLoraStatus(LOGALL);			
		}
	} // end of Modem-Loop

	delay(SCANDELAY);					// wait per loop in ms -> no time to loose.
} // while(run forever)

	// this point will never be reached

	// end LoRa Gateway sessions
	delete gwhwset[0].modem;
	delete gwhwset[1].modem;

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
int BeeIoTParse(beeiotpkg_t * mystatus){
int pkglen;
int rc=0; 
int ndid =0;
int idx=0;
int	needaction=0;
int loopid=0;
int nparam = 0;
char *ptr;					// ptr to next sensor parameter field

	// assumed we have the format: [GWID + SendID + PkgIdx + length] + [data set]
	//                                        4 Byte header         + [DatIdx + DateSTamp, ...]
	// done by ISR: mystatus = (beeiotpkg_t*) message -> now we take the payload as real data format

	needaction = 0;
	pkglen = BIoT_HDRLEN + mystatus->hd.frmlen + BIoT_MICLEN;
	cp_nb_rx_rcv++;		// statistics for BIoTApp(): # received pkgs. +1

	// now check real data: MIC & Header first -> JOIN server
	rc = JS_ValidatePkg(mystatus);
	if(rc < 0){
	// no valid Node packet header -> Header IDs unknown or Node not joined yet
		if (rc == -1 || rc == -2){ // -1: -> GWID;  -2: -> NodeID
			BHLOG(LOGLORAW) printf("  BeeIoTParse: Unknown Node/GW (NodeID:0x%02X -> GWID: 0x%02X) -> packet rejected\n", 
					(unsigned char)mystatus->hd.sendID, (unsigned char)mystatus->hd.destID);		
			// for test purpose: dump payload in hex format
			BHLOG(LOGLORAR) hexdump((unsigned char*) mystatus, MAX_PAYLOAD_LENGTH);
			BHLOG(LOGLORAR) printf("\n");
			// ignore this package: no action
			return(rc);
		}else if (rc == -3) {
			BHLOG(LOGLORAW) printf("  BeeIoTParse: Node registered but not joined yet (NodeID:0x%02X) -> Request a RE-JOIN\n", 
				(unsigned char)mystatus->hd.sendID);
			needaction = CMD_REJOIN;	// Request JOIN command from Node
			BeeIoTFlow(needaction, mystatus, mystatus->hd.sendID-NODEIDBASE, 0);
			// but we return a positive status
			rc=-3;
			return(rc);
		}else if(rc == -4){
			BHLOG(LOGLORAW) printf("  BeeIoTParse: Node joined but using wrong GWID 0x%02X -> should rejoin\n", 
				(unsigned char)mystatus->hd.destID);					
			return(rc);			
		}else if (rc == -5){
			// wrong Framelen detected -> request RETRY
			BHLOG(LOGLORAW) printf("  BeeIoTParse: Wrong Framelen 0x%02X -> requesting a RETRY\n", 
				(unsigned char)mystatus->hd.frmlen);					
			BeeIoTFlow(CMD_RETRY, mystatus, mystatus->hd.sendID-NODEIDBASE, 0);
			return(rc);
		}
	}
	ndid = rc;	// we received Node-ID of curr. Pkg from JS
	// For (RE-)JOIN Request ndid is set to 0 by JS !
	cp_nb_rx_ok++;			// incr. # of correct received packages	
	
	// Validate NwServer process flow:
	// - If we already know this BeeIoT Package: check the msgid
	if(ndid != 0){ // to be skipped during (RE-)JOIN request
		if(mystatus->hd.index == NDB[ndid].msg.idx){	// do we have already received this pkgid from this node ?
			BHLOG(LOGLORAW) printf("  BeeIoTParse: package (%d) duplicated -> package dropped ->send ACK\n\n", (unsigned char) mystatus->hd.index); // yes	
			// may be last ACK got lost, do it once again
			needaction = CMD_ACK;	// already known, but o.k.
			BeeIoTFlow(needaction, mystatus, ndid, 0);
			// but we return a positive status
			rc=0;
			return(rc);
		}
	}	
	// is it a direct command for the NwServer ?
	switch (mystatus->hd.cmd){
	case CMD_NOP:// intentionally do nothing but ACK
		// intended to do nothing -> test message for communication check
		// for test purpose: dump payload in hex format
		BHLOG(LOGLORAW) printf("  BeeIoTParse: NOP -> Send ACK");	
		BHLOG(LOGLORAR) hexdump((unsigned char*) mystatus, pkglen);
		needaction = CMD_ACK;	// we got it
		BeeIoTFlow(needaction, mystatus, ndid, 0);
		BHLOG(LOGLORAW) printf("=> Done.\n");	
		rc= 0;	
		break;
	case CMD_JOIN:// Node to register for data collection
		BHLOG(LOGLORAW) printf("  BeeIoTParse: Node JOIN Requested, MsgID: 0x%02X\n", (unsigned char)mystatus->hd.index);
		BHLOG(LOGLORAR) hexdump((unsigned char*) mystatus, pkglen);

		rc = JS_RegisterNode(mystatus, 0);	// evaluate WLTable and create NDB[] entry
		if(rc >= 0){
			ndid = rc;	// get index of new NDB-entry 
			BHLOG(LOGLORAW) printf("  BeeIoTParse: New Node%i Send CONFIG (with new channel data) as ACK\n",ndid);
			// give node some time to recover from SendMsg before 
			delay(MSGRX1DELAY);
			needaction = CMD_CONFIG; // acknowledge JOIN request
			BeeIoTFlow(needaction, mystatus, ndid, 0);
			BHLOG(LOGLORAW) printf("  => JOIN Done.\n");	
			rc= 0;	
		}else{
			BHLOG(LOGLORAW) printf("  BeeIoTParse: JOIN failed (rc=%i)\n", rc);
			rc=-7;
		}
		break;
	case CMD_REJOIN:	// Node was registered -> reactivate for data collection
		BHLOG(LOGLORAW) printf("  BeeIoTParse: Node REJOIN Requested, MsgID: 0x%02X\n", (unsigned char) mystatus->hd.index);
		BHLOG(LOGLORAR) hexdump((unsigned char*) mystatus, pkglen);

		rc = JS_RegisterNode(mystatus, 0);	// evaluate WLTable and NDB[] entry
		if(rc >= 0){	// successfully reactivated
			ndid = rc;	// get idx of known NDB-entry 
			BHLOG(LOGLORAW) printf("  BeeIoTParse: Node Reactivated: Just Send CONFIG (with new channel data) as ACK\n");
			// give node some time to recover from SendMsg before 
			delay(MSGRX1DELAY);
			needaction = CMD_CONFIG; // acknowledge JOIN request
			BeeIoTFlow(needaction, mystatus, ndid, 0);
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
		rc = JS_AppProxy( (int) ndid, (char*) mystatus->data, (byte) mystatus->hd.frmlen);

		// Was FramePayload complete ?
		if(rc == -1){   // wrong parameter set => request a resend of same message: RETRY
                    // lets acknowledge received package to sender to send it again
                    // ToDo: check for endless flow loop: only once
                    needaction = CMD_RETRY;
                    BeeIoTFlow(needaction, mystatus, ndid, 0);  // header can be reused
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
        BeeIoTFlow(needaction, mystatus, ndid, 0);  // send ACK in sync mode

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
		BHLOG(LOGLORAW) printf("  BeeIoTParse: SDLOG Save command: CMD(%d) not supported yet\n", (unsigned char) mystatus->hd.cmd);		
		// for test purpose: dump paload in hex format
		BHLOG(LOGLORAR) hexdump((unsigned char*) mystatus, pkglen);

		// ToDO: we can send ACK or SD data packages directly ?!
		needaction = CMD_ACK;	// o.k. we got it
		BeeIoTFlow(needaction, mystatus, ndid, 0);
		BHLOG(LOGLORAW) printf("  BeeIoTParse: ACK sent\n");		
		rc=-6; // but not supported yet
		break;
		
	default:	// unknown CMD for BEEIoT Protocol
		BHLOG(LOGLORAW) printf("  BeeIoTParse: unknown CMD(%d)\n", (unsigned char) mystatus->hd.cmd);		
		// for test purpose: dump payload in hex format
		BHLOG(LOGLORAR) hexdump((unsigned char*)mystatus, pkglen);
//		needaction = 0;	// no
//		BeeIoTFlow(needaction, mystatus, 0);
		rc= -99;
		break;
	}

	return(rc);
} // end of BeeIoTParse()


void BeeIoTFlow(u1_t action, beeiotpkg_t * pkg, int ndid, int async){
// default = sync mode -> async = 0
beeiotpkg_t  actionpkg;
beeiot_header_t	*pack;	// ACK requires header only	-> reuse of pkg message field
beeiot_cfg_t	*pcfg;	// CONFIG has HD + CData	-> reuse of pkg message field
nodedb_t		*pndb;
byte pkglen;
int count;
struct tm		*tval;	// values of current timestamp
Radio * Modem;

	BHLOG(LOGLORAR) printf("  BeeIoTFlow: New Action(async=%i): Send %s to Node:0x%02X\n", 
					async, beeiot_ActString[action], NODEIDBASE+(byte)ndid);
	switch (action){
	case CMD_NOP:	// fall thru by intention ==> like CMD_ACK
	case CMD_RETRY: // fall thru by intention ==> like CMD_ACK
	case CMD_ACK:	// by intention do nothing but ACK
		// give node some time to recover from SendMsg before 
		delay(RXACKGRACETIME);

		pack = (beeiot_header_t*) & actionpkg;
		pndb = &NDB[ndid];	// get Cfg init data from NDB entry of this node
		// lets acknowledge action cmd related to received package to sender
		pack->destID = pkg->hd.sendID;	// The BeeIoT node is the messenger
		pack->sendID = (u1_t) pndb->nodecfg.gwid; // New sender: its me
		pack->cmd = action;				// get our action command
		pack->index = pkg->hd.index;		// get last pkg msgid
		pack->frmlen = 0;				// send BeeIoT header for ACK only
		pkglen = BIoT_HDRLEN+BIoT_MICLEN; // just the BeeIoT header + MIC
		break;

	case CMD_REJOIN:	// Send  a simple REJOIN request (reuse ACK Format buffer)
		pack = (beeiot_header_t*) & actionpkg;
		pndb = &NDB[ndid];	// get Cfg init data from NDB entry of this node
		// lets acknowledge action cmd related to received package to sender
		pack->destID = pkg->hd.sendID;	// The BeeIoT node is the messenger
		pack->sendID = (u1_t) gwid0;;	// New sender: its me on JOIN channel
		pack->cmd = action;				// get our action command
		pack->index = pkg->hd.index;	// get last pkgid
		pack->frmlen = 0;				// send BeeIoT header for ACK only
		pkglen = BIoT_HDRLEN+BIoT_MICLEN; // just the BeeIoT header + MIC
		break;

	case CMD_CONFIG: // Send runtime config params as assiged to node
		// give node some time to recover from SendMsg before 
		delay(RXACKGRACETIME);
		pcfg = (beeiot_cfg_t*) & actionpkg;	// use generic pkg space for CONFIG pkg
		// setup pkg header
		pcfg->hd.destID = pkg->hd.sendID;	// The BeeIoT node is the messenger
		pcfg->hd.sendID = (u1_t)gwid0;	// New sender: its me on JOIN channel
		pcfg->hd.cmd = action;			// get our action command
		pcfg->hd.index = pkg->hd.index;	// get last pkg msgid
		pcfg->hd.frmlen = sizeof(devcfg_t); // 

		pndb = &NDB[ndid];	// get Cfg init data from NDB entry of this node
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
		return;	
	}
	
	// Do final TX for all cases:
	BeeIotTXFlag = 0;						// spawn IRQ-> Userland TX flag
	if(actionpkg.hd.sendID == GWIDx){ // if default JOIN channel addressed
		Modem = gwset[0].modem;
	}else if(cfgini->loranumchn >= 2){	// multi channel mode ? -> use next channel
		Modem = gwset[1].modem;
	}else{ // use default channel always (also for non default GW-ID)
		Modem = gwset[0].modem;
	}

	Modem->starttx((byte *)&actionpkg, pkglen );	// send LoRa pkg	
	if (async) {							// wait till bytes are out ?
		// no, but spend some grace time for the radio to be save
		delayMicroseconds(150);		// wait for 150 max. send bytes in Tsymb=1ms (SF7)
	} else {						// lets wait till TXDone-ISR reports TX completion
		count = 0;
		while (!BeeIotTXFlag) {		// check for TX compl. flag processed by ISR routine
			delay(MAX_PAYLOAD_LENGTH);	// Give ISR also some time to validate TXDone
			// ToDO: timeout if TX fails in a certain time 
			if(++count == MAXTXTO){
				BHLOG(LOGLORAR) printf("  BeeIoTFlow: TX Time Out !\n");
				Modem->setopmode(OPMODE_SLEEP);	// reset Modem FIFO and go sleeping
				break;
			}
		}
	}

	//		PrintLoraStatus(LOGALL);

	return;
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

