//*******************************************************************
// File:	NwSrv.h  
// Project: https://github.com/mchresse/BeeIoTGW
// Author:	MCHREsse
// Created on 28. April 2020, 22:03
//
// Description:
// BeeIoT-GateWay - BIoTWAN Network service class definition
//
//----------------------------------------------------------
// Copyright (c) 2020-present, Randolph Esser
// All rights reserved.
// This file is distributed under the BSD-3-Clause License
// The complete license agreement can be obtained at: 
//     https://github.com/mchresse/BeeIoTGW/license
// For used 3rd party open source see also Readme_OpenSource.txt
//*******************************************************************
#ifndef NWSRV_H
#define NWSRV_H

using namespace std;

// frequence of MsgQueue polling / modem; finally defines min. reaction time on pkg IRQ
// set to 0 for multichannel mode with many nodes
#define NWSSCANDELAY	100		// [ms]

class NwSrv {
public:
	NwSrv(gwbind_t &gwtab, int nmodem); // Constructor
	~NwSrv(void);						  // Destructor
	
	// Start scanning of all discovered Modems -> endless (!)
	// detected LoRa Pkgs from ISR are forwarded to BeeIoTParse()
	// Finally Lora Modems reset to RXCont mode again
	int NwNodeScan(void);
	
	// get number of activated and served modems by NwSrv
	int	NwSrvModems(void);

protected:

private:
	gwbind_t	&gwt;
	modemcfg_t	*gwhwset;
	int			mactive;		// Max. number of detected active modems for RX/TX
	struct timeval now;			// current tstamp used each time a time check is done
	char	TimeString[128];	// contains formatted Timestamp string

	// remember beacon id of last pkg as q-check
	int last_beaconid;

	// Parsing of retrieved raw Lora pkg data for BIotWAN conformity
	int  BeeIoTParse(MsgBuffer * msg);
	// Actor of final Parse-result -> serving BIoTWAN Pkg flow protocol
	int  BeeIoTFlow(u1_t action, beeiotpkg_t * pkg, int ndid, bool async);	
	// process beacon msg
	int	NwSBeacon(u1_t mid, int ndid, MsgBuffer *msg);

}; // end of class NwSrv
#endif /* NWSRV_H */

