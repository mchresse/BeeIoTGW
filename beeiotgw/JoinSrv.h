//*******************************************************************
// File:	NwSrv.h  
// Project: https://github.com/mchresse/BeeIoTGW
// Author:	MCHREsse
// Created on 29. April 2020, 10:54
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
#ifndef JOINSRV_H
#define JOINSRV_H

using namespace std;

#define UP_LINK   0   // DIR: Client -> GW
#define DOWN_LINK 1   // DIR: GW -> Client

//******************************************************************
// NodeWLTable -> manually preset for reference of join requests
typedef struct{
	byte	nodeid;		// Node ID (base: NODEIDBASE + 0...MAXNODES)
	byte	gwid;		// GW ID   (base: GWIDx - 0...MAXGW) 
	byte	mid;		// Modem ID(base: 0...mactive) -> NwSrv (initial)
	byte	AppEUI[LENJOINEUI];	// corresponding unique AppEUI of serving App
	byte	DevEUI[LENDEVEUI];	// Node unique DevEUI
	byte	reportfrq;	// [min] Frequency of Status data reports
	bool	joined;		// == 0 Node has not been joined; ==1 joined
	byte	chncfg;		// channel config idx to txchntab[]
	int		wcalib;		// weight cell calibration value +/-
} nodewltable_t;

// NodeDB[] typeset
typedef struct{
	msghd_t msg;		// runtime data of last received message of node -> use this mid for answer pkg
	nodewltable_t * pwltab; // ptr to corresponding WL node table
	joinpar_t	nodeinfo;	// Node info descriptor
	devcfg_t	nodecfg;	// parameter set for node side runtime config.
	byte		AppSKey[LENAPPSKEY];
	byte		NwSKey[LENNWSKEY];
	devaddr_t	DevAddr;
	byte		middef;		// Default Modem ID from WLTab[] corresponding to nodecfg.gwid
	// but GW is always in slave mode -> only pkg sent as answer pkg (see msg.mid above)
	int			wcalib;		// weight cell calibration value +/- 
} nodedb_t;

class JoinSrv {
//******************************************************************************
public:
	nodedb_t NDB[MAXNODES];	// if 'NDB[x].nodeinfo.version == 0' => empty entry
	// -> typeset see beelora.h

	JoinSrv(gwbind_t &gwtab, int nmodem); // Constructor
	~JoinSrv(void);						  // Destructor
	
	//*****************************************
	// JOINSrv API member function prototypes
	void	JS_Cfg2Wlt		(void);	// Init WLTab[] by cfgini GWset data

	// Initialize complete NDB[] table
	void	JS_InitNDB		(void);
	// 
	int		JS_RegisterNode	(beeiotpkg_t * joinpkg);
	int		JS_ValidatePkg	(beeiotpkg_t* mystatus);

//******************************************************************************
protected:

//******************************************************************************
private:
	gwbind_t	& gwt;
//	modemcfg_t	* gwhwset;
	int			mactive;		// Max. number of detected active modems for RX/TX
	struct timeval now;			// current tstamp used each time a time check is done
	char		TimeString[128];// contains formatted Timestamp string
	


//*****************************************
// JOINSrv local member function prototypes

	// verify Pkg data integrity by MIC code
	int JS_ValidateMic(beeiotpkg_t* mystatus, uint8_t mode, int ndid);

	//******************************************************************************
	// compare 2 binary streams with given length "binlen".
	// rc=0 if equal: if not equal, rc provides # of equal bytes found in stream
	inline int JS_ByteStreamCmp(byte * bina, byte * binb, int binlen){
		int i;
		for (i=0; i<binlen; i++){
			if(bina[i]!= binb[i])
				return(i+1);
		}
		return(0);
	}

}; // end of class JoinSrv
#endif /* JOINSRV_H */
