//*******************************************************************
// File:	BIoTApp.h  
// Project: https://github.com/mchresse/BeeIoTGW
// Author:	MCHREsse
// Created on 30. April 2020, 10:49
//
// Description:
// BeeIoT-GateWay - BIoTWAN App services class definition AppSrv
// Provides class definition for the following App services:
//	- BIoT	 Parsing BIoT Node Log Status for Bee Hive monitoring
//			 LogData is parsed and converted for forwarding via BeeCSV()
//			 to ext. webspace or MQTT service (t.b.d.)
//	- GH	 Simply economy monitoring of plant houses (t.b.d.)
//	- Turtle Simple economy monitoring of turtle houses	(t.b.d.)
//
//----------------------------------------------------------
// Copyright (c) 2020-present, Randolph Esser
// All rights reserved.
// This file is distributed under the BSD-3-Clause License
// The complete license agreement can be obtained at: 
//     https://github.com/mchresse/BeeIoTGW/license
// For used 3rd party open source see also Readme_OpenSource.txt
//*******************************************************************
#ifndef BIOTAPP_H
#define BIOTAPP_H

using namespace std;

// number of defined JoinEUIDs and corresponding AppServer Functions
#define MAXBIOTAPP	3	

class AppSrv {
//*************************************************************************
public:
	 AppSrv(gwbind_t &gwtab); // Constructor
	~AppSrv(void);				 // Destructor
	
	// route App message to JoinEUI related App refernced by NDB[ndid]
	int	AppProxy(int ndid, char * framedata, byte framelen, int mid);

//------------------------------------------------------------
protected:

//------------------------------------------------------------
private:
	//*****************************************
	// BIoTApp API member function prototypes
	int AppBIoT		(int ndid, char* data, byte len, int mid);		// Bee Weight Scale App
	int AppTurtle	(int ndid, char* data, byte len, int mid);		// Turtle House Control App
	int AppGH		(int ndid, char* data, byte len, int mid);		// GreenHouse Control App

	//*****************************************
	// BIoTApp local member function prototypes
	// 
	// Prepare JSON formatted data field for TTN upload
	void UploadPkg	(char * msg, int pkglen, int SNR, byte rssi, int mid);

	// Prepare  status update package in Byte stream format for TTN upload
	void sendstat	(int mid);

	// ToDo: send prepared message to TTn server
	void sendudp	(char * msg, int length);

	
	//*****************************************
	// BIoTApp private prototypes:

	// private ptr to the global BIoT GW instances & settings
	gwbind_t	&gwt;
	modemcfg_t	* gwhwset;

	// Status Housekeeping
	int			nnodes;			// Number of detected active BIoT Nodes
	struct timeval now;			// current tstamp used each time a time check is done
	char		TimeString[128];// contains formatted Timestamp string

	// JOIN-EUI reference table & App jump table for JS_AppProxy()
	const char TJoinEUI[MAXBIOTAPP][LENJOINEUI] = {
			BIoT_EUID, 
			TURTLE_EUID, 
			GH_EUID,
	};
	
	// BIoTApp Helper functions
	//******************************************************************************
	// compare 2 binary streams with given length "binlen".
	// rc=0 if equal: if not equal, rc provides # of equal bytes found in stream
	inline int ByteStreamCmp(byte * bina, byte * binb, int binlen){
		int i;
		for (i=0; i<binlen; i++){
			if(bina[i]!= binb[i])
				return(i+1);
		}
		return(0);
	}

}; // end of class AppSrv

#endif /* BIOTAPP_H */

