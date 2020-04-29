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



class JoinSrv {
//******************************************************************************
public:
	nodedb_t NDB[MAXNODES];	// if 'NDB[x].nodeinfo.version == 0' => empty entry
	// -> typeset see beelora.h

	JoinSrv(modemcfg_t *gwtab, int nmodem); // Constructor
	~JoinSrv(void);							// Destructor
	
	//*****************************************
	// JOINSrv API member function prototypes

	// Initialize complete NDB[] table
	void	JS_InitNDB		(void);
	// 
	int		JS_RegisterNode	(beeiotpkg_t * joinpkg);
	int		JS_ValidatePkg	(beeiotpkg_t* mystatus);
	int		JS_AppProxy		(int ndid, char * framedata, byte framelen, int mid);

//******************************************************************************
protected:

//******************************************************************************
private:
	modemcfg_t *gwhwset;
	int			mactive;		// Max. number of detected active modems for RX/TX
	struct timeval now;			// current tstamp used each time a time check is done
	char		TimeString[128];// contains formatted Timestamp string
	


	//*****************************************
	// JOINSrv local member function prototypes

	// Compares 2 binary streams with given length "binlen".
	// rc=0 if equal: if not equal, rc provides # of equal bytes found in stream
	int	JS_ByteStreamCmp	(byte * bina, byte * binb, int binlen);

}; // end of class JoinSrv
#endif /* JOINSRV_H */
