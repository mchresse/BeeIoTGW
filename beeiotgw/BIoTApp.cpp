/*******************************************************************************
* The "BIoTApp" Module is distributed under the New BSD license:
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
// Some parts of this Module (Functions: UploadPkg(), SendStatus(), SendUDP() )
// are inspired by the project "Single Channel gateway"
// from T. Thelkamp. Therefore this License disclaimer:
/*******************************************************************************
 *
 * Copyright (c) 2015 Thomas Telkamp
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 *******************************************************************************/
/*******************************************************************************
 * Sub Module BIoTApp.cpp
 *
 *******************************************************************************
 */

#include <iostream>
#include <cstdint>
#include <cstring>
#include <sys/time.h>

#include <net/if.h>
#include <arpa/inet.h>
#include <linux/sockios.h>

#include "base64.h"
#include "BeeIoTWan.h"
#include "beeiot.h"
#include "gwqueue.h"
#include "beelora.h"
#include "radio.h"
#include "JoinSrv.h"
#include "BIoTApp.h"

//*****************************************************************
// global variables from main.cpp
extern unsigned int	lflags;		// BeeIoT log flag field (main.cpp)

// Central Database of all measured values and runtime parameters
extern dataset	bhdb;			// central beeIoT data DB
extern configuration * cfgini;	// ptr. to struct with initial parameters



// to retrieve local LAN Port MAC address (defined in main ()
extern struct sockaddr_in si_other;
extern int		s, slen;
extern struct ifreq ifr;




// value fields used by all App services
static struct	timeval now; // current tstamp used each time a time check is done
static char	TimeString[128]; // contains formatted Timestamp string of each loop(in main())

// for generic FTP xfer via curl lib
// further FTP addr. setting to be retrieved from config.ini
static int	curlcount;		// count curl-lib ftp xfer calls for debugging reasons

extern int beecsv(dataset * data);
extern int beelog(char *comment);

//******************************************************************
// Settings for TTN Upload in JSON format via UDP:

//*****************************************
// TTN Connection: TTN servers
// TODO: use host names and dns
#define TTNSERVER1  "54.72.145.119"   // The Things Network: croft.thethings.girovito.nl
#define TTNSERVER1b "54.229.214.112"  // The Things Network: croft.thethings.girovito.nl
#define TTNSERVER2  "192.168.1.10"    // local
#define TTNPORT		1700              // The port on which to send data

// TTN Upload buffer settings
#define TX_BUFF_SIZE		2048
#define STATUS_SIZE			1024
// ----------------------------------------
#define PROTOCOL_VERSION	1
#define PKT_PUSH_DATA		0
#define PKT_PUSH_ACK		1
#define PKT_PULL_DATA		2
#define PKT_PULL_RESP		3
#define PKT_PULL_ACK		4

// Gateway GPS location (preset manually, if no GPS access is active)
static float lat=0.0;
static float lon=0.0;
static int   alt=0;

// User Info SendStat() settings 
// (todo: retrieved/upd. from user config.ini)
static char platform[24]    = "Single Channel Gateway";	// platform definition
static char email[40]       = "";                       // used for contact email
static char description[64] = "BIoT WAN Gateway";		// used for free form description



//***************************************************************************
// AppSrv Constructor
AppSrv::AppSrv(gwbind_t &gwtab): gwt(gwtab) {
	// Init housekeeping status params -> upd. during runtime
	nnodes = 0;
	TimeString[0] = 0;
} // end of AppSrv()

// AppSrv Destructor
AppSrv::~AppSrv(void){
} // end of ~AppSrv()


//******************************************************************************
// AppProxy()
// Gets JOIN EUI from NDB by given Node id and search for assigned APP-function 
// in the App proxy table. 
// in case of a hit -> assigned function pointer is called and return code is 
// forwarded to caller.
//
// INPUT:
//	ndid refers to valid WLTab entry
//	framedata
//	framelen
//	mid
// RETURN:
//  -98		wrong NDB index as input value
//	-99		wrong/unsupported APP JOINEUI
//	 rc		return code of App
//	
// 


int AppSrv::AppProxy(int ndid, char * framedata, byte framelen, int mid){
	int idx, rc;
	
	if(ndid > MAXNODES){
		BHLOG(LOGBIOT) printf("  AppProxy: wrong NDB[] index %i\n", ndid);
		return(-98);
	}	
	// get ptr. to NDB[] entry of current node via NwSrv
	nodedb_t *pndb = & gwt.jsrv->NDB[ndid];

	for( idx=0; idx< MAXBIOTAPP; idx++){
		// compare JoinEUI from NDB with local TJoimEUI reference table entry (by 8 byte len)
		if(ByteStreamCmp( (byte*)& TJoinEUI[idx][0], (byte*)& pndb->nodeinfo.joinEUI, LENJOINEUI) == 0)
			break;		// we have a hit -> known DevEUI found		
	}

	// call assigned APP Server function with given Frame-payload
	switch(idx){
		case 0:	// Bee weight cell App
			rc = AppBIoT (ndid, framedata, framelen, mid);
			break;
		case 1:	// Turtle House control App
			rc = AppTurtle (ndid, framedata, framelen, mid);
			break;
		case 2:	// GreenHouse Control App
			rc = AppGH (ndid, framedata, framelen, mid);
			break;
		default:	// no match in APP function EUID table found
			BHLOG(LOGBH) printf("  JS_AppProxy: wrong App JOIN EUI idx %i\n", idx);
			return(-99);
	}

	return(rc);
}



//******************************************************************************
// AppBIoT()
// Analyzing BIoT Frame payloads for Weight Scale lifecycle
// return:
//  0       Data processed successfully -> ACK can be sent
//	1		add. Message for RX1 available in TXBUFFER
// -1       wrong number of parsed sensor parameters -> Retry needed
// -2       problems with CSV file creation/concatenation
// -99      Wrong input data ; call rejected
int AppSrv::AppBIoT	(int ndid, char* data, byte len, int mid){
    int rc = 0;
    int idx;
    int nparam;

	nodedb_t *pndb = &gwt.jsrv->NDB[ndid]; // ptr to NDB[ndid]

    if(data && len == 0){    // prevent NULL ptr. and bitlen=0
        BHLOG(LOGBIOT) printf("  AppBIoT: Wrong input data (%p, %i)\n", data, (int)len);
        return(-99);        // wrong input data
    }

	// get current timestamp
	gettimeofday(&now, 0);
	strftime(TimeString, 80, "%Y-%m-%d %H:%M:%S", localtime(&now.tv_sec));
    BHLOG(LOGBH) printf("\n  AppBIoT: %s -Processing Sensor data (len:%i) of BIoT-Node: 0x%02X\n", 
				TimeString, (int)len, (unsigned char) pndb->nodecfg.nodeid);

    idx = 0; // start with first entry (by now the only one)

    data[len-2]=0;	// limit string by EOL -> set new end marker (and cut off EOL:0D0A)
    BHLOG(LOGBIOT) printf("    Status Node0x%02X: %s ",(unsigned char) pndb->nodecfg.nodeid, data); // to be checked if it is a string
    BHLOG(LOGBIOT) printf("%iBy.\n", (int) len);

    // parse sensor status stream for whitespace chars conflicting with sscanf()
    for(int i=0; i<strlen(data); i++){
            if(data[i]== ',' | data[i]== ';')
                data[i] = ' ';	// and replace them by space
    }
    // parse all BeeIoT-WAN status parameters from stream to bhdb-dataset row
    nparam = sscanf(data, "%s %s %f %f %f %f %f %f %f %f %f %d #%d %s", 
            bhdb.date,
            bhdb.time, 
            & bhdb.dlog[idx].HiveWeight,
            &(bhdb.dlog[idx].TempExtern),
            &(bhdb.dlog[idx].TempIntern),
            &(bhdb.dlog[idx].TempHive),
            &(bhdb.dlog[idx].TempRTC),
            &(bhdb.dlog[idx].ESP3V),		// in V !
            &(bhdb.dlog[idx].Board5V),		// in V !
            &(bhdb.dlog[idx].BattCharge),   // in V !
            &(bhdb.dlog[idx].BattLoad),		// in V !
            &(bhdb.dlog[idx].BattLevel),
            &(bhdb.dlog[idx].index),		// Data Msg Index (not Lora Pkg Index !)
            &(bhdb.dlog[idx].comment) );

    // is Sensor parameter set complete ?
    if(nparam != BEEIOT_STATUSCNT || nparam == EOF){
            BHLOG(LOGBH) printf("  AppBIoT: Sensor-Status incomplete, found %i status parameters (expected %i)\n", nparam, BEEIOT_STATUSCNT);
            // lets acknowledge received package to sender to send it again
            // ToDo: prevent endless Retry loop: only once
            rc = -1;
            return(rc);
    }

    // We have received a complete sensor parameter Set => Store it !
	// complete DB datarow by Node information

		// bhdb.dlog[].timestamp    = timestamp of node sample data
		if(strlen(bhdb.date) + strlen(bhdb.time) <= LENTMSTAMP){
			sprintf(bhdb.dlog[idx].timeStamp, "%s %s", bhdb.date, bhdb.time);
		} 		
		// bhdb.date + bhdb.time = timestamp of last APP processing 
		strftime(bhdb.date, LENDATE, "%Y/%M/%D", localtime(&now.tv_sec));
		strftime(bhdb.time, LENTIME, "%H:%M:%S", localtime(&now.tv_sec));

		bhdb.packid     = (int)0 + (short)pndb->nodeinfo.frmid[0]; // get sequential index of payload packages
        bhdb.loopid     = bhdb.dlog[idx].index;     // MsgID: sensor scan loop counter of sensor data set
        bhdb.ipaddr[0]  = 0;                        // no IP yet
        bhdb.NodeID     = pndb->nodecfg.nodeid;		// BIoT network identifier to expand CSV File name
		bhdb.dlog[idx].HiveWeight += pndb->wcalib;	// calibrate weight cell value to final one

        memcpy((byte*)&bhdb.BoardID, (byte*) &pndb->nodeinfo.devEUI, sizeof(uint64_t));

	// 1. Forward Data to local Web Service 
	//        BHLOG(LOGBH) printf("  AppBIoT: Forward Data as CSV file to local WebPage\n");		
        rc = beecsv(&bhdb); // save it in CSV format first
        if(rc != 0){
            BHLOG(LOGBH) printf("  AppBIoT: CSV file not found (rc=%i)\n", rc);		
            rc= -2;
            // ToDo: error recovery of CSV file handling
            return(rc);     // frame data accepted, but not forwarded/stored
        }

	// Further Options: Forward dataset to remote WebSpace as CSV file, via FTP, or via MQTT or to TTN
/*
	// send TTN status message each STATUSLOOP seconds (untested code)
        gettimeofday(&nowtime, NULL);
        uint32_t nowseconds = (uint32_t)(nowtime.tv_sec);
        if (nowseconds - lasttime >= STATUSLOOP) {
            lasttime = nowseconds;
            // forward the data via TTN
            sendstat(mid);
        }
*/
        gwt.cp_up_pkt_fwd++;	// Statistic: incr. # of forwarded status packages

		// ToDo: if add. Message to Node: prepare in TXBUFFER here
		// rc = 1; / Let NwSrv send it back to Node
		
		return(rc);
} // end of AppBIoT()


//******************************************************************************
// AppTurtle()
// Analyzing Turtle House Sensor data
//
int AppSrv::AppTurtle (int ndid, char* data, byte len, int mid){
	BHLOG(LOGTURTLE) printf("  AppTurtle: Processing new Sensor Status data (len:%i)\n", (int)len);

	return(0);
}

//******************************************************************************
// AppGH Main Function
// Analyzing GH House Sensor data
//
int AppSrv::AppGH (int ndid, char* data, byte len, int mid){
	BHLOG(LOGGH) printf("  AppGH: Processing new Sensor Status data (len:%i)\n", (int)len);

	return(0);
}



//******************************************************************************
// Prepare JSON formatted data field for TTN upload
// INPUT:
//  *msg	ptr on raw data field
//	pkglen	length of data in msg field
//	SNR		Signal-Noise-Ratio in dBm
//	rssi	RSSI quality value of retrieved package
//	mid		modem ID of retrieving message channel of this pkg
void AppSrv::UploadPkg( char * msg, int pkglen, int SNR, byte rssi, int mid ) {

	char buff_up[TX_BUFF_SIZE]; // buffer to compose the upstream packet
	byte buf64[512];			// TTN payload encoding buffer

            BHLOG(LOGLORAW) printf(" TTN Upload Pkg to Server: Length: %i\n",(int)pkglen);
            BHLOG(LOGLORAR) printf("RSSI: %d, ", (unsigned char) rssi);
            BHLOG(LOGLORAR) printf("SNR: %li, ",  SNR);
			
			// encode payload for network transfer
			int j;
            j = bin_to_b64((uint8_t *)msg, pkglen, (char *)(buf64), 341);
            //fwrite(buf64, sizeof(char), j, stdout);
            
            int buff_index=0;

            /* gateway <-> MAC protocol variables */
            //static uint32_t net_mac_h; /* Most Significant Nibble, network order */
            //static uint32_t net_mac_l; /* Least Significant Nibble, network order */

            /* pre-fill the data buffer with fixed fields */
            buff_up[0] = PROTOCOL_VERSION;
            buff_up[3] = PKT_PUSH_DATA;

            /* process some of the configuration variables */
            //net_mac_h = htonl((uint32_t)(0xFFFFFFFF & (lgwm>>32)));
            //net_mac_l = htonl((uint32_t)(0xFFFFFFFF &  lgwm  ));
            //*(uint32_t *)(buff_up + 4) = net_mac_h;
            //*(uint32_t *)(buff_up + 8) = net_mac_l;

            buff_up[4] = (unsigned char)ifr.ifr_hwaddr.sa_data[0];
            buff_up[5] = (unsigned char)ifr.ifr_hwaddr.sa_data[1];
            buff_up[6] = (unsigned char)ifr.ifr_hwaddr.sa_data[2];
            buff_up[7] = 0xFF;
            buff_up[8] = 0xFF;
            buff_up[9] = (unsigned char)ifr.ifr_hwaddr.sa_data[3];
            buff_up[10] = (unsigned char)ifr.ifr_hwaddr.sa_data[4];
            buff_up[11] = (unsigned char)ifr.ifr_hwaddr.sa_data[5];

            /* start composing datagram with the header */
            uint8_t token_h = (uint8_t)rand(); /* random token */
            uint8_t token_l = (uint8_t)rand(); /* random token */
            buff_up[1] = token_h;
            buff_up[2] = token_l;
            buff_index = 12; /* 12-byte header */

            // TODO: tmst can jump is time is (re)set, not good.
            struct timeval now;
            gettimeofday(&now, 0);
            uint32_t tmst = (uint32_t)(now.tv_sec*1000000 + now.tv_usec);

            /* start of JSON structure */
            memcpy((void *)(buff_up + buff_index), (void *)"{\"rxpk\":[", 9);
            buff_index += 9;
            buff_up[buff_index] = '{';
            ++buff_index;
            j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE-buff_index, "\"tmst\":%u", tmst);
            buff_index += j;
            j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE-buff_index, 
					",\"chan\":%1u,\"rfch\":%1u,\"freq\":%.6lf", 0, 0, (double)(gwt.modem[mid]->getchannelfrq()) / 1000000);
            buff_index += j;
            memcpy((void *)(buff_up + buff_index), (void *)",\"stat\":1", 9);
            buff_index += 9;
            memcpy((void *)(buff_up + buff_index), (void *)",\"modu\":\"LORA\"", 14);
            buff_index += 14;
            /* Lora datarate & bandwidth, 16-19 useful chars */
            switch (gwt.modem[mid]->getspreading()) {
            case SF7:
                memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF7", 12);
                buff_index += 12;
                break;
            case SF8:
                memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF8", 12);
                buff_index += 12;
                break;
            case SF9:
                memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF9", 12);
                buff_index += 12;
                break;
            case SF10:
                memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF10", 13);
                buff_index += 13;
                break;
            case SF11:
                memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF11", 13);
                buff_index += 13;
                break;
            case SF12:
                memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF12", 13);
                buff_index += 13;
                break;
            default:
                memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF?", 12);
                buff_index += 12;
            }
			switch (gwt.modem[mid]->getband()){
			case BW125:
				memcpy((void *)(buff_up + buff_index), (void *)"BW125\"", 6);
				break;
			case BW250:
				memcpy((void *)(buff_up + buff_index), (void *)"BW250\"", 6);
				break;
			case BW500:
				memcpy((void *)(buff_up + buff_index), (void *)"BW500\"", 6);
				break;
			default:
				break;
			}
            buff_index += 6;

			switch (gwt.modem[mid]->getcoding()){
			case CR_4_5:
				memcpy((void *)(buff_up + buff_index), (void *)",\"codr\":\"4/5\"", 13);
				break;
			case CR_4_6:
				memcpy((void *)(buff_up + buff_index), (void *)",\"codr\":\"4/6\"", 13);
				break;
			case CR_4_7:
				memcpy((void *)(buff_up + buff_index), (void *)",\"codr\":\"4/7\"", 13);
				break;
			case CR_4_8:
				memcpy((void *)(buff_up + buff_index), (void *)",\"codr\":\"4/8\"", 13);
				break;
			default:
				break;
			}
            buff_index += 13;

            j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE-buff_index, ",\"lsnr\":%li", SNR);
            buff_index += j;

            j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE-buff_index, ",\"rssi\":%d,\"size\":%u", rssi, pkglen);
            buff_index += j;

            memcpy((void *)(buff_up + buff_index), (void *)",\"data\":\"", 9);
            buff_index += 9;
            j = bin_to_b64((uint8_t *)msg, pkglen, (char *)(buff_up + buff_index), 341);
            buff_index += j;
            buff_up[buff_index] = '"';
            ++buff_index;

            /* End of packet serialization */
            buff_up[buff_index] = '}';
            ++buff_index;
            buff_up[buff_index] = ']';
            ++buff_index;
            /* end of JSON datagram payload */
            buff_up[buff_index] = '}';
            ++buff_index;
            buff_up[buff_index] = 0; /* add string terminator, for safety */

            BHLOG(LOGLORAW) printf("  Send rxpk update: %s\n", (char *)(buff_up + 12)); /* DEBUG: display JSON payload */

            //send the messages
            sendudp(buff_up, buff_index);

            fflush(stdout);
} // received a message


//*****************************************************************************
// Prepare  status update package in Byte stream format
void AppSrv::sendstat(int mid) {
    static char status_report[STATUS_SIZE]; // status report buffer
    char stat_timestamp[24];
    time_t t;

    int stat_index=0;

    /* pre-fill the data buffer with fixed fields */
    status_report[0] = PROTOCOL_VERSION;
    status_report[3] = PKT_PUSH_DATA;

    status_report[4] = (unsigned char)ifr.ifr_hwaddr.sa_data[0];
    status_report[5] = (unsigned char)ifr.ifr_hwaddr.sa_data[1];
    status_report[6] = (unsigned char)ifr.ifr_hwaddr.sa_data[2];
    status_report[7] = 0xFF;
    status_report[8] = 0xFF;
    status_report[9] = (unsigned char)ifr.ifr_hwaddr.sa_data[3];
    status_report[10] = (unsigned char)ifr.ifr_hwaddr.sa_data[4];
    status_report[11] = (unsigned char)ifr.ifr_hwaddr.sa_data[5];

    /* start composing datagram with the header */
    uint8_t token_h = (uint8_t)rand(); /* random token */
    uint8_t token_l = (uint8_t)rand(); /* random token */
    status_report[1] = token_h;
    status_report[2] = token_l;
    stat_index = 12; /* 12-byte header */

    /* get timestamp for statistics */
    t = time(0);
    strftime(stat_timestamp, sizeof stat_timestamp, "%F %T %Z", gmtime(&t));

    int j = snprintf((char *)(status_report + stat_index), STATUS_SIZE-stat_index, "{\"stat\":{\"time\":\"%s\",\"lati\":%.5f,\"long\":%.5f,\"alti\":%i,\"rxnb\":%u,\"rxok\":%u,\"rxfw\":%u,\"ackr\":%.1f,\"dwnb\":%u,\"txnb\":%u,\"pfrm\":\"%s\",\"mail\":\"%s\",\"desc\":\"%s\"}}", 
					stat_timestamp, lat, lon, (int)alt, gwt.cp_nb_rx_rcv, gwt.cp_nb_rx_ok, gwt.cp_up_pkt_fwd, (float)0, 0, 0,
					platform,email,description);
    stat_index += j;
    status_report[stat_index] = 0; /* add string terminator, for safety */

    BHLOG(LOGLORAW) printf("stat update: %s\n", (char *)(status_report+12)); /* DEBUG: display status */

    //send the update
    sendudp(status_report, stat_index);
}


//*****************************************************************************
// Send message pack to TTN network 
void AppSrv::sendudp(char *msg, int length) {

	// shortcut for test purpose
BHLOG(LOGLORAW) printf("sendudp(): len=%i <%s> (deactivated!)\n", length, msg);
BHLOG(LOGLORAW) hexdump((unsigned char*) msg, length);
	return;

//send the update
#ifdef SERVER1
    inet_aton(SERVER1 , &si_other.sin_addr);
    if (sendto(s, (char *)msg, length, 0 , (struct sockaddr *) &si_other, slen)==-1)
    {
        die("sendto()");
    }
#endif

#ifdef SERVER2
    inet_aton(SERVER2 , &si_other.sin_addr);
    if (sendto(s, (char *)msg, length , 0 , (struct sockaddr *) &si_other, slen)==-1)
    {
        die("sendto()");
    }
#endif
}

