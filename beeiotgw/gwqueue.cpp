/*******************************************************************************
* The "GwQueue" Module is distributed under the New BSD license:
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
* GwQueue.cpp is released under the New BSD license. Go to the project
* home page for more info: https://github.com/beeiot
********************************************************************************
*/
/*******************************************************************************
 * Sub Module GwQueue.cpp
 *
 *******************************************************************************
 */
#include <iostream>
#include <cstring>

#include "BeeIoTWan.h"
#include "beelora.h"
#include "beeiot.h"
#include "gwqueue.h"	// using STL container classes
#include "radio.h"
  
using namespace std;

extern unsigned int	lflags;               // BeeIoT log flag field


//*****************************************************************+
// Init_GwQueue()
// create new GwQueue for all Modems
/*
MsgQueue * init_gwqueue(int mid){	
	GwQueue = new MsgQueue;	// instantiate GW Queue
//	GwQueue->PrintStatus();
	return(GwQueue);	// Return Ptr on GwQueue object
}
*/

//*****************************************************************+
// Class Members of MsgQueue
// Control of MsgBuffer items in a prio-queue
// The MsgQueue Constructor
MsgQueue::MsgQueue(){
	mq = new std::queue<MsgBuffer>;  // get a new Queue instance
	if(!mq){	// no Queue received
		throw (int) EX_MSGQU_INIT;		// Exc.ID: RADIO Init failed -> break;
	}

	BHLOG(LOGQUE) printf("  MsgQueue: New empty MsgQueue created\n");
}

// The MsgQueue destructor
MsgQueue::~MsgQueue(){
	while(!mq->empty()){
		mq->pop();
	}
	BHLOG(LOGQUE) printf("  MsgQueue: MsgQueue removed\n");
}

// create and add MsgBuffer to PrioQueue-end: FiFo
// sort by ModemID: 0 = high prio
// replace for IsrIdx 
void MsgQueue::PushMsg(MsgBuffer & mbuf){
	int mid = mbuf.getmid();
	byte pkgid = mbuf.getpkgid();
	BHLOG(LOGQUE) printf("  MsgQueue: Push Msg[%i] ID:%i\n",mid, pkgid);

	mq->push(mbuf);	// add after last: FiFo
}

// remove MsgBuffer obj. from PrioQueue-start: FiFo
// Replace for SrvIdx
void MsgQueue::PopMsg(){
	if(mq->empty()) 
		return;

	MsgBuffer & mbuf = GetMsg();

	int mid = mbuf.getmid();
	byte pkgid = mbuf.getpkgid();
	BHLOG(LOGQUE) printf("  MsgQueue: Pop Msg[%i] ID:%i from Queue\n", mid, pkgid);

	mq->pop();	// remove first element: FiFo
}

// Deliver # of elements in queue
int MsgQueue::MsgQueueSize(){
	return(mq->size());
}

// get MsgBuffer from PrioQueue-start: FiFo
// Delivers RD/WR Reference to message entity for manipulation (e.g. by ISR)
MsgBuffer & MsgQueue::GetMsg(){
		return(mq->front());
}

// Print MsgBuffer-Queue Status: as one line with 'newline'
int MsgQueue::PrintStatus(){
	if(mq->empty()){
		printf("  MsgQueue: Length:%i, Msg0: None\n", 
				mq->size());
	}else{
		MsgBuffer & msg = GetMsg();
		printf("  MsgQueue: Length:%i, Modem%i->MsgID:%i\n", 
				mq->size(),(int)msg.getmid(), (int)msg.getpkgid());
	}
	return(mq->size());
}

		
//*****************************************************************+
// Class: Message Buffer
// holds all Data and control values of a received LoRa message
// Used as entity for PrioQueue-class

// Constructor & Destructor:
MsgBuffer::MsgBuffer(int modemid, int rssi, byte snr){
	msghd.mid		= modemid;
	msghd.pkgid		= 0;
	msghd.rssi		= rssi;
	msghd.snr		= snr;
	msghd.retries	= 0;
	msghd.ack		= 0;
	pkg = new beeiotpkg_t;	// reserve Msg buffer space
	if(!pkg){	// no Pkg buffer received
		throw (int) EX_MSGQU_INIT;		// Exc.ID: RADIO Init failed -> break;
	}
	BHLOG(LOGQUE) printf("  MsgBuffer[%i]: New MsgBuffer for id:%i created\n", getmid(), getpkgid());
}

MsgBuffer::~MsgBuffer(){
	BHLOG(LOGQUE) printf("  MsgBuffer[%i]: Delete MsgBuffer %i\n", getmid(), getpkgid());
//	delete pkg;			// release msg buffer
}

// overload < Operator (for prio qeueue sort)
bool operator < (MsgBuffer & msga, MsgBuffer & msgb){
	int mida = msga.getmid();
	int midb = msgb.getmid();
	return mida < midb;
}

// Get MsgHd Params in detail:
int	MsgBuffer::getmid(void){
	return(msghd.mid);	
}	
byte MsgBuffer::getpkgid(void){
	return(msghd.pkgid);
}
byte MsgBuffer::getretries(void){
	return(msghd.retries);
}
bool MsgBuffer::getack(void){
	return(msghd.ack);
}
int	MsgBuffer::getrssi(void){
	return(msghd.rssi);
}
int	MsgBuffer::getsnr(void){
	return(msghd.snr);
}

// Raw Copy MsgBuffer to User Package buffer
// return amount of copied data in byte
int	MsgBuffer::getpkg(beeiotpkg_t * pkgx){
	memcpy((byte*)pkgx, (byte*) pkg, sizeof(beeiotpkg_t));
	return(sizeof(beeiotpkg_t));
}	

// Copy LoRa SX-FiFo Package to MsgBuffer directly out of the SPI FiFo Register
// using: Radio::HAL: SPI-WRbuffer cop)
int	MsgBuffer::setpkgfifo(Radio * Modem, byte sxreg, int dlen){
	if(dlen > sizeof(beeiotpkg_t)) 
		dlen = sizeof(beeiotpkg_t);	// copy max. size of dest. buffer

	Modem->readBuf(sxreg, (byte *) pkg, (byte) dlen);	// call as friend function to Radio class
	BHLOG(LOGQUE) printf("  MsgBuffer[%i]: Rd SX FiFo - %d Bytes/n", (int)Modem->mset.modemid, dlen);
	return(dlen); // return # of copied bytes
}


// Print pkg bytewise in one line in hex format
void MsgBuffer::printpkg(int dlen = 0){
	if(dlen > sizeof(beeiotpkg_t)) 
		dlen = sizeof(beeiotpkg_t);	// copy max. size of dest. buffer
	byte * ppkg = (byte*) pkg;
	for (int i=0; i<dlen; i++){
		printf("%02X ", ppkg[i]);
	}
}

// Set Package ID & Ack Flag
void MsgBuffer::setpkgid_ack(byte pkgid, bool ack){
	msghd.pkgid		= pkgid;
	msghd.ack		= ack;
}

// Simply initialize all MsgBuffer Header params at once
void MsgBuffer::setmsghd(msghd_t & msghdx){
	msghd.mid		= msghdx.mid;
	msghd.pkgid		= msghdx.pkgid;
	msghd.ack		= msghdx.ack;
	msghd.retries	= msghdx.retries;
	msghd.rssi		= msghdx.rssi;
	msghd.snr		= msghdx.snr;	
	BHLOG(LOGQUE) printf("  MsgBuffer[%i]: Init Buffer - PkgID%i, RSSI:%i, SNR:%i/n",
			(int)msghd.mid, (int)msghd.pkgid, (int)msghd.rssi, (int)msghd.snr);
}	// set complete MsgHD at once
