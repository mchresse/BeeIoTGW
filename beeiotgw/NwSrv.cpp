/*******************************************************************************
* The "NwSrv" Module is distributed under the New BSD license:
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


#include "loraregs.h"

#define BEEIOT_ACTSTRINGS
#include "BeeIoTWan.h"
#undef BEEIOT_ACTSTRINGS

#include "beelora.h"
#include "beeiot.h"


extern unsigned int	lflags;               // BeeIoT log flag field

/*******************************************************************************
 *
 * BIoTWAN LoRA protocol Constants: Please customize for your new network:
 * (see also settings in beelora.h)
 *
 *******************************************************************************/



//******************************************************************************
// Global NodeDB[] for Node registration via JOIN command
#define STATUSLOOP	1000		// in main(): wait loop delay in ms

extern nodedb_t NDB[];			// if 'NDB[x].nodeinfo.version == 0' => empty entry
// -> typeset see beelora.h; instance in JoinSrv.cpp

// current LoRa MAC layer settings -> radio.cpp
extern long freq; // in Mhz! (868.1)
extern bw_t bw;
extern sf_t	sf;
extern cr_t cr;
extern int irqlevel;

static const int SPI0	= 0;		// use RPi SPI channel 0
static const int SPIFRQ = 500000;	// use SPI channel Frequency 0,5MHz


extern byte currentMode;		// see radio.c
extern struct timeval now;		// current tstamp used each time a time check is done
extern unsigned long txstart;	// tstamp when TX Mode was entered
extern unsigned long txend;		// Delta: now - txstart
extern unsigned long rxtime;	// tstamp when last rx package arrived
char	TimeString[128];		// contains formatted Timestamp string of each loop(in main())
// extern const char * rxloraOMstring[];  // string field for current OpMode Status (radio.cpp)

// Initialize BIoT WAN default adresses for joining nodes
byte	gwid	= GWIDx;		// get curr. ID of this GW Instance
byte	nodeid	= NODEIDBASE;	// get curr. ID of nodeID we are talking with

// RX/TX Queues
byte    BeeIotRXFlag =0;            // Semaphor for received message(s) 0 ... MAXRXPKG-1
byte    BeeIotTXFlag =0;            // Semaphor for a single sent message
byte    RXPkgIsrIdx  =0;            // index on next RX Queue Package for ISR callback Write
byte    RXPkgSrvIdx  =0;            // index on next RX Queue Package for Service Routine Read/serve
beeiotpkg_t MyRXData[MAXRXPKG];     // RX Pkg Queue: received messages for userland processing (undecoded)

extern uint32_t cp_nb_rx_ok;			// # of correct received packages -> Statistics for BIoTApp()

// Central Database of all measured values and runtime parameters
extern dataset			bhdb;		// central beeIoT data DB
extern configuration	cfgini;		// runtime parameter init source
	
//******************************************************************************
// local Function prototypes
int  NwNodeScan		(void);
int  BeeIoTParse	(beeiotpkg_t * mystatus);
void BeeIoTFlow		(u1_t action, beeiotpkg_t * pkg, int ndid, int async);

// in JoinSrv.cpp:
extern int	RegisterNode	(beeiotpkg_t * joinpkg, int async);
extern int	JS_ValidatePkg	(beeiotpkg_t* mystatus);
extern int	ByteStreamCmp	(byte * bina, byte * binb, int binlen);


/******************************************************************************
 * NwNodeScan()
 ******************************************************************************/
int NwNodeScan () {
    struct timeval nowtime;
    uint32_t lasttime;
	int rc;

	printf("  NwS: BeeIoT-WAN NwServer: Start Node Scan\n");

	// get current timestamp
	gettimeofday(&now, 0);
	strftime(TimeString, 80, "%d-%m-%y %H:%M:%S", localtime(&now.tv_sec));
	 BHLOG(LOGBH) printf("  NwS<%s>: Setup started for BIoTWAN V%d.%d.%d\n", 
		TimeString,  (int)BIoT_VMAJOR, (int)BIoT_VMINOR, (int)BIoT_VMINSUB);
    
	pinMode(LORAcs,	 OUTPUT);
    pinMode(LORArst, OUTPUT);
    pinMode(LORAdio0,INPUT);
    pinMode(LORAdio1,INPUT);
    pinMode(LORAdio2,INPUT);
	digitalWrite(LORAcs, HIGH);
	digitalWrite(LORArst, HIGH);
    wiringPiSPISetup(SPI0, SPIFRQ);

	// Init NodeDB[] for new registrations:
	for(int i=0; i<MAXNODES; i++){
		NDB[i].nodeinfo.vmajor =0; // free entry if 'NDB[x].nodeinfo.vmajor == 0'
		NDB[i].nodeinfo.vminor =0;
		NDB[i].nodeinfo.devEUI[0] =0;
		NDB[i].nodeinfo.devEUI[1] =0;
		NDB[i].nodeinfo.devEUI[2] =0;
		NDB[i].nodeinfo.devEUI[3] =0;
		NDB[i].nodecfg.gwid		= gwid;
		NDB[i].nodecfg.nodeid	= NODEIDBASE + i;	// a bit critical here: NODEIDx is defined statically
		NDB[i].nodecfg.vmajor	= BIoT_VMAJOR;	// Major + Minor version: V01.00
		NDB[i].nodecfg.vminor	= BIoT_VMINOR;
		NDB[i].nodecfg.verbose	= 0;
		NDB[i].nodecfg.channelidx =0;
		NDB[i].nodecfg.freqsensor = (int) 60;	// [min] reporting frequence of status pkg.
		NDB[i].msg.ack = 0;
		NDB[i].msg.idx = 0;
		NDB[i].msg.pkg = (beeiotpkg_t*)NULL;
		NDB[i].msg.retries = 0;
	}
	
	// Initialize BIoT WAN default addresses for joining nodes
	// For Node session communication, NDB[] values have to be used (as init by WLTab).
	gwid	= GWIDx;
	nodeid	= NODEIDBASE;

	// Preset RX QUEUE: MyRXData[MAXRXPKG]
	BeeIotRXFlag = 0;
	BeeIotTXFlag = 0;
	RXPkgSrvIdx  = 0;
	RXPkgIsrIdx	 = 0;

	// Preset LoRa Modem Configuration	
	// Modem Setting for RX Mode: 
    SetupLoRa();	// -> enter SLEEP Mode	
//	PrintLoraStatus(LOGALL);

	myisr_init();		// assign ISRs to IRQ Port

	BHLOG(LOGLORAR)PrintLoraStatus(LOGALL);
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
	BHLOG(LOGLORAW) printf("  NwS: LoRa Modem Setup finished\n");

	// Start LoRa Mode continuous read loop
	startrx(RXMODE_SCAN, 0);	// RX in RX_CONT Mode 

	// get current timestamp
	gettimeofday(&now, 0);
	strftime(TimeString, 80, "%d-%m-%y %H:%M:%S", localtime(&now.tv_sec));
	BHLOG(LOGBH) printf("  NwS:********************************************************************\n");
    BHLOG(LOGLORAW) printf("  NwS<%s>:  Waiting for LoRa Node Packets in Contiguous Mode (%.6lf Mhz - SF%i)\n",
		TimeString, (double)freq/1000000, (unsigned char)sf+6);

	int counter =0;
	// run forever: wait for incoming packages via radio_irq_handler()
    while(1) {
		// ISR waiting for rising DIO0 level -> starting receivepacket() directly
		while(BeeIotRXFlag){	// Do we have a new package in the RX Queue ?
			// check RX Queue BeeIoT WAN Status and process package accordingly
			// get current timestamp
			gettimeofday(&now, 0);
			strftime(TimeString, 80, "%d-%m-%y %H:%M:%S", localtime(&now.tv_sec));
			BHLOG(LOGLORAR) printf("  NwS<%s>: New RX Packet: Parsing  Queue-MyRXData[%i] (BeeIoTRXFlag:%i)\n", 
					TimeString, (int)RXPkgSrvIdx, (int)BeeIotRXFlag);
			rc = BeeIoTParse(&MyRXData[RXPkgSrvIdx]);
			if(rc < 0){
				BHLOG(LOGLORAW) printf("  NwS: Parsing of RXMsg[%i] failed rc=%i\n", (int)RXPkgSrvIdx, rc);
				// ToDo: Exit/Recover Code here ???
				BHLOG(LOGLORAR) PrintLoraStatus(LOGALL);		
			}
//			opmode(OM_LORA_STANDBY); // should be done by ISR already; just to be sure

			// switch to next Queue element
			if(++RXPkgSrvIdx >= MAXRXPKG){  // RX Queue end reached ?
			  BHLOG(LOGLORAR) printf("  NwS: RX Buffer end reached, switch back to buffer[0]\n");
			  RXPkgSrvIdx=0;  // wrap around
			}
			BeeIotRXFlag--;   // however,  we can release one more RX Queue buffer
	
			BHLOG(LOGLORAR) printf("  NwS: New RX-Queue Status: SrvIdx:%i, IsrIdx:%i, RXFlag:%i, IRQlevel=%i\n",
				(int)RXPkgSrvIdx, (int)RXPkgIsrIdx, (int)BeeIotRXFlag, (int)irqlevel);
			counter =0;
		}
		if((currentMode & OM_LORA_RX_CONT)!= OM_LORA_RX_CONT){
			// Start LoRa MOde continuous read loop
			BHLOG(LOGLORAR) printf("  NwS: Enter RX_Cont Mode (SF%i - %.6lf Mhz)\n", (int) sf+6,(double)freq/1000000);
			startrx(RXMODE_SCAN, 0);	// RX in RX_CONT Mode (Beacon Mode)
			BHLOG(LOGBH) printf("  NwS: ********************************************************************\n");
//			PrintLoraStatus(LOGALL);
		}

		delay(STATUSLOOP);					// wait per loop in ms -> no time to loose.
/*		printf("%i ", counter++);
		if ((counter%25) == 0)
			printf("\n");
*/    }
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
int nid =0;
int idx=0;
int	needaction=0;
// string dataMessage;
int loopid=0;
int nparam = 0;
char *ptr;					// ptr to next sensor parameter field

	// assumed we have the format: [GWID + SendID + PkgIdx + length] + [data set]
	//                                        4 Byte header         + [DatIdx + DateSTamp, ...]
	// done by ISR:
	// mystatus = (beeiotpkg_t*) message;	// now we take the paylod for real data format
	needaction = 0;
	pkglen = BIoT_HDRLEN + mystatus->hd.frmlen + BIoT_MICLEN;

	// now check real data: MIC & Header first -> JOIN server
	rc = JS_ValidatePkg(mystatus);
	if(rc < 0){
	// no valid Node packet header -> node IDs unknown or not joined yet
		if (rc == -1 || rc == -2){ // -1 -> GWID;  -2 -> NodeID
			BHLOG(LOGLORAW) printf("  BeeIoTParse: Unknown Node/GW (NodeID:0x%02X -> GWID: 0x%02X) -> packet rejected\n", 
					(unsigned char)mystatus->hd.sendID, (unsigned char)mystatus->hd.destID);		
			// for test purpose: dump payload in hex format
			BHLOG(LOGLORAR) hexdump((unsigned char*) mystatus, MAX_PAYLOAD_LENGTH);
			BHLOG(LOGLORAR) printf("\n");
			// ignore this package: no action
			return(rc);
		}else if (rc == -3) {
			BHLOG(LOGLORAW) printf("  BeeIoTParse: Node registered but not joined yet (NodeID:0x%02X) -> packet rejected\n", 
				(unsigned char)mystatus->hd.sendID);
			return(rc);
		}else if(rc == -4){
			BHLOG(LOGLORAW) printf("  BeeIoTParse: Node joined but using wrong GWID 0x%02X -> should rejoin\n", 
				(unsigned char)mystatus->hd.destID);					
			return(rc);			
		}else if (rc== -5){
			// wrong Framelen detected -> request RETRY
			BeeIoTFlow(CMD_RETRY, mystatus, mystatus->hd.index-NODEIDBASE, 0);
			return(rc);
		}
	}
	int ndid = rc;	// we received Node-ID of curr. Pkg from JS
	// For JOIN Request ndid ==0 !
	cp_nb_rx_ok++;			// incr. # of correct received packages	
	// Validate NwServer process flow:
	// - Do we already know this BeeIoT Package: check the msgid
	if(ndid != 0){ // to be skipped during JOIN request
		if(mystatus->hd.index == NDB[ndid].msg.idx){	// do we have already received this pkgid from this node ?
			BHLOG(LOGLORAW) printf("  BeeIoTParse: package (%d) duplicate -> package dropped ->send ACK\n\n", (unsigned char) mystatus->hd.index); // yes	
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
	case CMD_NOP:// intentially do nothing but ACK
		// intended tp do nothig -> test message for communication check
		// for test purpose: dump paload in hex format
		BHLOG(LOGLORAW) printf("  BeeIoTParse: NOP -> Send ACK");	
		BHLOG(LOGLORAR) hexdump((unsigned char*) mystatus, pkglen);
		needaction = CMD_ACK;	// we got it
		BeeIoTFlow(needaction, mystatus, ndid, 0);
		BHLOG(LOGLORAW) printf("=> Done.\n");	
		rc= 0;	
		break;
	case CMD_JOIN:// Node to register for data collection
		BHLOG(LOGLORAW) printf("  BeeIoTParse: Node JOIN Requested, MsgID: 0x%02X\n", mystatus->hd.index);
		BHLOG(LOGLORAR) hexdump((unsigned char*) mystatus, pkglen);

		rc = RegisterNode(mystatus, 0);	// evaluate WLTable and create NDB[] entry
		if(rc >= 0){
			ndid = rc;	// get idx of new NDB-entry 
			BHLOG(LOGLORAW) printf("  BeeIoTParse: New Node: Send CONFIG (with new channel data) as ACK\n");
			needaction = CMD_CONFIG; // acknowledge JOIN request
			BeeIoTFlow(needaction, mystatus, ndid, 0);
			BHLOG(LOGLORAW) printf("  => JOIN Done.\n");	
			rc= 0;	
		}else{
			BHLOG(LOGLORAW) printf("  BeeIoTParse: JOIN failed (rc=%i)\n", rc);
			rc=-7;
		}
		break;
	case CMD_LOGSTATUS: // New BeeIoT node sensor data set received
		// First save dataset to BHDB
		BHLOG(LOGLORAW) printf("  BeeIoTParse: Sensor-Status received\n");	
		loopid  = 0; // start with first entry (by now the only one)
		bhdb.packid = mystatus->hd.index;	// get sequential index of payload packages
		bhdb.ipaddr[0]	= 0;			// no IP yet
		bhdb.BoardID	= 0;			// no board ID yet
		bhdb.loopid		= loopid;		// reset sequential index of sample data set

		mystatus->data[mystatus->hd.frmlen-2]=0;	// than limit string -> set new end marker (and cut off EOL:0D0A)
		BHLOG(LOGLORAW) printf("  Status: %s ", mystatus->data); // to be checked if it is a string
		BHLOG(LOGLORAW) printf("%iBy.\n", pkglen);
		
		// parse sensor status stream for whitespace chars conflicting with sscanf()
		for(idx=0; idx<strlen(mystatus->data); idx++){
			if(mystatus->data[idx]== ',' | mystatus->data[idx]== ';')
				mystatus->data[idx] = ' ';	// and replace them by space
		}
		// parse all BeeIoT-WAN status parameters from stream to bhdb-dataset row
		nparam = sscanf(mystatus->data, "%s %s %f %f %f %f %f %f %f %f %f %d #%d %s", 
			bhdb.date,
			bhdb.time, 
			& bhdb.dlog[loopid].HiveWeight,
			&(bhdb.dlog[loopid].TempExtern),
			&(bhdb.dlog[loopid].TempIntern),
			&(bhdb.dlog[loopid].TempHive),
			&(bhdb.dlog[loopid].TempRTC),
			&(bhdb.dlog[loopid].ESP3V),			// in V !
			&(bhdb.dlog[loopid].Board5V),		// in V !
			&(bhdb.dlog[loopid].BattCharge),	// in V !
			&(bhdb.dlog[loopid].BattLoad),		// in V !
			&(bhdb.dlog[loopid].BattLevel),
			&(bhdb.dlog[loopid].index),
			&(bhdb.dlog[loopid].comment) );
			if(strlen(bhdb.date) + strlen(bhdb.time) <= LENTMSTAMP){
				sprintf(bhdb.dlog[loopid].timeStamp, "%s %s", bhdb.date, bhdb.time);
			}
		// is parameter set complete ?
		if(nparam != BEEIOT_STATUSCNT || nparam == EOF){
			BHLOG(LOGLORAW) printf("  BeeIoTParse: Sensor-Status incomplete, found # of status parameters: %i (expected %i)\n", nparam, BEEIOT_STATUSCNT);
			// lets acknowledge received package to sender to send it again
			// ToDo: check for endless flow loop: only once
			bhdb.packid = mystatus->hd.index-1;	// we expect same msgid again
			needaction = CMD_RETRY;
			BeeIoTFlow(needaction, mystatus, ndid, 0);
			rc= -4;
		}else{
			// save dataset to CSV file
			rc = beecsv(&bhdb);
			if(rc != 0){
				BHLOG(LOGLORAW) printf("  BeeIoTParse: CSV file not found (rc=%i)\n", rc);		
				rc= -5;
				// ToDo: error recovery of CSV file
			}
			needaction = CMD_ACK;	// no saved data, but o.k.
			BeeIoTFlow(needaction, mystatus, ndid, 0);
			delay(MSGRESWAIT);	// distance between 2 messages

			BHLOG(LOGLORAW) printf("  BeeIoTParse: ACK sent\n", rc);		

			// detect addon packages to send in RX1
			BHLOG(LOGLORAW) printf("  BeeIoTParse: No RX1 Msg\n", rc);		
//			needaction = CMD_CONFIG;
//			BeeIoTFlow(needaction, mystatus, 0);	// hand over mystatus for header data
			rc=0;
		}
		break;
	case CMD_GETSDLOG: // SD LOG data package received
		BHLOG(LOGLORAW) printf("  BeeIoTParse: SDLOG Save command: CMD(%d) not supported yet\n", (unsigned char) mystatus->hd.cmd);		
		// for test purpose: dump paload in hex format
		hexdump((unsigned char*) mystatus, pkglen);

		// ToDO: we can send ACK or SD data packages directly ?!
		needaction = CMD_ACK;	// o.k. we got it
		BeeIoTFlow(needaction, mystatus, ndid, 0);
		BHLOG(LOGLORAW) printf("  BeeIoTParse: ACK sent\n", rc);		
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

	BHLOG(LOGLORAR) printf("  BeeIoTFlow: New Action(async=%i): Send %s to Node:0x%02X\n", 
					async, beeiot_ActString[action], NODEIDBASE+(byte)ndid);
	switch (action){
	case CMD_NOP:	// fall thru by intention ==> like CMD_ACK
	case CMD_RETRY: // fall thru by intention ==> like CMD_ACK
	case CMD_ACK:	// intentially do nothing but ACK
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

	case CMD_CONFIG: // Send runtime config param set to node
		// give node some time to recover from SendMsg before 
		delay(RXACKGRACETIME);
		pcfg = (beeiot_cfg_t*) & actionpkg;	// use generic pkg space for CONFIG pkg
		// setup pkg header
		pcfg->hd.destID = pkg->hd.sendID;	// The BeeIoT node is the messenger
		pcfg->hd.sendID = (u1_t)gwid;	// New sender: its me
		pcfg->hd.cmd = action;			// get our action command
		pcfg->hd.index = pkg->hd.index;	// get last pkg msgid
// ToDO fill dcfg with data from NDB[] -> how to get NDB index ?
		pcfg->hd.frmlen = sizeof(devcfg_t); // 

		pndb = &NDB[ndid];	// get Cfg init data from NDB entry of this node
		pcfg->cfg.channelidx= pndb->nodecfg.channelidx;	// we start with default Channel IDX
		pcfg->cfg.gwid		= pndb->nodecfg.gwid;		// store predefined GW
		pcfg->cfg.nodeid	= pndb->nodecfg.nodeid;
		pcfg->cfg.freqsensor= pndb->nodecfg.freqsensor;	// [min] loop time of sensor status reports in Seconds
		pcfg->cfg.verbose	= pndb->nodecfg.verbose;	// reserved definition
		pcfg->cfg.vmajor	= pndb->nodecfg.vmajor;		// get Version of BIoT protocol of node
		pcfg->cfg.vminor	= pndb->nodecfg.vminor;		// gives room for backward support stepping		
		pcfg->cfg.nonce		= pndb->nodecfg.nonce;

		pkglen = BIoT_HDRLEN + sizeof(devcfg_t) + BIoT_MICLEN;	// Cfg-Payload + BIOT Header
		BHLOG(LOGLORAR) hexdump((byte*) &actionpkg, pkglen);
	break;
	default: // don't know what to do  
		return;	
	}
	// Do TX
	BeeIotTXFlag = 0;					// spawn IRQ-> Userland TX flag
	starttx((byte *)&actionpkg, pkglen );	// send LoRa pkg
	if (async) {						// wait till bytes are out ?
		// no, but spend some grace time for the radio to be save
		delayMicroseconds(150); // wait for 150 max. send bytes in Tsymb=1ms (SF7)
	} else {				// lets wait till TXDone-ISR reports TX completion
		count = 0;
		while (!BeeIotTXFlag) {		// check for TX compl. flag processed by ISR routine
			delay(MAX_PAYLOAD_LENGTH);	// Give ISR also some time to validate TXDone
			// ToDO: timeout if TX fails in a certain time 
			if(++count == MAXTXTO){
				BHLOG(LOGLORAR) printf("  BeeIoTFlow: TX Time Out !\n");
				opmode(OM_LORA_SLEEP);	// reset Modem FIFO
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
