/*******************************************************************************
* The "BIoTApp" Module is distributed under the New BSD license:
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <iostream>
#include <sys/time.h>

#include <net/if.h>
#include <arpa/inet.h>
#include <linux/sockios.h>

#include "base64.h"
#include "loraregs.h"
#include "BeeIoTWan.h"
#include "beelora.h"
#include "beeiot.h"


extern unsigned int	lflags;               // BeeIoT log flag field
//******************************************************************
// Set my GPS location (preset manually, if no GPS access
float lat=0.0;
float lon=0.0;
int   alt=0;

/* Informal status fields for TTN Upload JSON package*/
static char platform[24]    = "Single Channel Gateway";  /* platform definition */
static char email[40]       = "";                        /* used for contact email */
static char description[64] = "";                        /* used for free form description */

byte receivedbytes;			// generic RX msg data len
byte message[256];			// generic RX msg buffer
byte buf64[512];			// payload encoding buffer

// to retrieve local LAN Port MAC address (defined in main ()
extern struct sockaddr_in si_other;
extern int s, slen;
extern struct ifreq ifr;


// current LoRa MAC layer settings -> radio.cpp
extern long freq; // in Mhz! (868.1)
extern bw_t bw;
extern sf_t	sf;
extern cr_t cr;

// ToDO: to be initialize/updated by radio.spp layer during rx/tx package handling
uint32_t cp_nb_rx_rcv	= 0; // # received packages
uint32_t cp_nb_rx_ok	= 0; // # of correct received packages
uint32_t cp_up_pkt_fwd	= 0; // # of sent status packages to TTN
uint32_t cp_nb_rx_bad;
uint32_t cp_nb_rx_nocrc;


//******************************************************************************
// local Function prototypes
void UploadPkg( char * msg, int pkglen, int SNR, byte rssi);
void sendudp(char *msg, int length);
void sendstat(void);


//******************************************************************************
// BIoTApp Main Function
// Analysing BIoT Frame payloads for Weight Scale lifecycle
//
void BIoTApp(void){
	/*
		// send TTN status message each STATUSLOOP seconds
        gettimeofday(&nowtime, NULL);
        uint32_t nowseconds = (uint32_t)(nowtime.tv_sec);
        if (nowseconds - lasttime >= STATUSLOOP) {
            lasttime = nowseconds;

			// here we could forward the data via TTN
            sendstat();
            cp_nb_rx_rcv = 0;		// reset # received packages
            cp_nb_rx_ok = 0;		// reset # of correct received packages
            cp_up_pkt_fwd = 0;		// reset # of sent status packages to TTN
        }
*/
	return;
}


void UploadPkg( char * msg, int pkglen, long int SNR, byte rssi ) {

            BHLOG(LOGLORAW) printf("  UploadPkg to Server: Length: %i\n",(int)pkglen);
            BHLOG(LOGLORAR) printf("RSSI: %d, ", (unsigned char) rssi);
            BHLOG(LOGLORAR) printf("SNR: %li, ",  SNR);
			
			// encode payload for network transfer
			int j;
            j = bin_to_b64((uint8_t *)msg, pkglen, (char *)(buf64), 341);
            //fwrite(b64, sizeof(char), j, stdout);

            char buff_up[TX_BUFF_SIZE]; /* buffer to compose the upstream packet */
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
					",\"chan\":%1u,\"rfch\":%1u,\"freq\":%.6lf", 0, 0, (double)freq/1000000);
            buff_index += j;
            memcpy((void *)(buff_up + buff_index), (void *)",\"stat\":1", 9);
            buff_index += 9;
            memcpy((void *)(buff_up + buff_index), (void *)",\"modu\":\"LORA\"", 14);
            buff_index += 14;
            /* Lora datarate & bandwidth, 16-19 useful chars */
            switch (sf) {
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
			switch (bw){
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

			switch (cr){
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


// Send status update packe to TTN networj in JSON format
void sendudp(char *msg, int length) {

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


// Prepare  status update package in JSON format
void sendstat() {
    static char status_report[STATUS_SIZE]; /* status report as a JSON object */
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
					stat_timestamp, lat, lon, (int)alt, cp_nb_rx_rcv, cp_nb_rx_ok, cp_up_pkt_fwd, (float)0, 0, 0,
					platform,email,description);
    stat_index += j;
    status_report[stat_index] = 0; /* add string terminator, for safety */

    BHLOG(LOGLORAW) printf("stat update: %s\n", (char *)(status_report+12)); /* DEBUG: display JSON stat */

    //send the update
    sendudp(status_report, stat_index);

}

