/* 
 * File:   gwqueue.h
 * Author: mchresse - Randolph Esser
 *
 * Created on 21. April 2020, 11:42
 */

#ifndef GWQUEUE_H
#define GWQUEUE_H

#include <queue>

#define MAXRXPKG	10			// RX Queue Len: Max. number of parallel processed RX packages

// Declaration of MsgBuffer Header
typedef struct {
	int			mid;		// modem ID of receiving gateway
    byte		idx;		// index of sent message: 0..255 (round robin)
    byte		retries;	// number of initiated retries
	bool		ack;		// ack flag 1 = message received
	int			rssi;		// RSSI value computed while receiving the frame [dBm]
	byte		snr;		// Raw SNR value given by the radio hardware in dB
    beeiotpkg_t *pkg;		// sent message struct -> BeeIotWAN.h
} beeiotmsg_t;

typedef struct {
	int			mid;		// modem ID of receiving gateway
    byte		pkgid;		// LoRa pkg index of sent message: 0..255 (round robin)
    byte		retries;	// number of initiated retries
	bool		ack;		// ack flag 1 = message received
	int			rssi;		// RSSI value computed while receiving the frame [dBm]
	byte		snr;		// Raw SNR value given by the radio hardware in dB
} msghd_t;

// forward declaration for friend function setpkgfifo()
class Radio;

//*****************************************************************+
// Class: Message Buffer
// holds all Data and control values of a received LoRa message
// Used as entity for PrioQueue-class
// Queue element item for RX packages
class MsgBuffer {
public:
	// Constructor & Destructor:
	MsgBuffer(int modemid,int rssi, byte snr);
	~MsgBuffer();

	// get MsgHd Params in detail
	int		getmid(void);	
	byte	getpkgid(void);
	byte	getretries(void);
	bool	getack(void);
	int		getrssi(void);
	int		getsnr(void);

	// get LoRa Package raw copied to pkgx user buffer
	int		getpkg(beeiotpkg_t * pkgx);	
	void	setmsghd(msghd_t & msghdx);	// set complete MsgHD at once
	void	setpkgid_ack(byte pkgid, bool ack);

	// set LoRa Package to MsgBuffer directly out of the SX FiFo (HAL: SPI buffer copy)
	int		setpkgfifo(Radio * Modem, byte sxreg, int dlen);
	
	void	printpkg(int dlen);

	// overload< Operator (for prio queue sort)
	friend bool operator < (MsgBuffer & msga, MsgBuffer & msgb);

protected:
	// none

private:	
	msghd_t		msghd;		// Control Header of message frame
    beeiotpkg_t *pkg;		// LoRa Package frame (for casts see -> BeeIotWAN.h)

}; // end of class MsgBuffer


//*****************************************************************+
// Prototype of Class: Message (Prio-)Queue
// Holds all Msg Buffer packages for all Gateways sorted by Modem ID
// -> Msg. with JOIN cmds have lower prio
class MsgQueue {
	public:
		MsgQueue(void);
		~MsgQueue(void);

		// add MsgBuffer to PrioQueue-end: FiFo
		// sort by ModemID: 0 = high prio
		void PushMsg(MsgBuffer & mbuf);

		// remove MsgBuffer from PrioQueue-start: FiFo
		void PopMsg(void);

		// get MsgBuffer from PrioQueue-start: FiFo
		// ToDo: Copy Constructor
		MsgBuffer & GetMsg(void);	

		// deliver # of elements in queue
		int MsgQueueSize(void);

		// Print msg Queue Status in one line + /n
		int PrintStatus(void);
		
	protected:
	private:
		std::queue<MsgBuffer> * mq;
		int mid;		// current modem ID Queue was assigned to

		// RX Queue buffer & control -> obsolete
//		byte    RXFlag;            // Semaphore for received message(s) 0 ... MAXRXPKG-1
//		byte    IsrIdx;            // WR index on next RX Queue Package for ISR callback Write
//		byte    SrvIdx;            // RD index on next RX Queue Package for BIoTParse() Service

}; // end of class MsgQueue

#endif /* GWQUEUE_H */

