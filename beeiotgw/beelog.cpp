/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/*******************************************************************************
 * File:   beelog
 * Author: Randolph Esser - Copyright 2017-2019
 * Created on 17. Januar 2017, 10:09
 * This file is part of the "beelog" program/project. 
 *
 * Creation of all Logfiles and reports to extern:
 * - Create log files as interface to further rpograms
 *		- BeeLog.txt, 
 *		- Daily CSV Format for measured raw data sets
 *		- BeeDays - statistic CSV file
 * - Add web notices to Webpage index file for service purpose
 *******************************************************************************
*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sched.h>
#include <string.h>
#include <sys/sysinfo.h>
#include <time.h>
#include <sys/time.h>

#include "BeeIoTWan.h"
#include "regslora.h"
#include "beeiot.h"

//******************************************************************************
// Global data reference section
extern unsigned int		lflags;		// BeeIoT log flag field
extern dataset			bhdb;
extern configuration	*cfgini;	// ptr. to struct with initial parameters

//******************************************************************************
// Local data declarations
static struct timeval	now;		// my current local time buffer
static char	TimeString[128];		// Timestamp object of current action
									// loop common for all Log files -> beelog()

/*******************************************************************************
* Function: beelog()
* Creation: 22.01.2017
* 
* Write LOG data in a special line format with time stamp into log file.
* Logfile path and name as defined in config.ini
*
* Input :
*	comment		string of logdata (/wo timestamp)
*
* Global:	
*	cfgini struct	struct of runtime parameters
*	beelog.log file
*
* Output:
*	return		 0 log data written to LOG file, 
*				-1 file could not be found/opened
*
*********************************************************************************
*/
int beelog(char * comment){
char	logline[LOGLINELEN];	// text buffer of one log line
FILE *	bhlog;					// file handle of logfile
char	logpath[1024];			// log file path buffer	


	// build logdata string ->  get current timestamp
	gettimeofday(&now, NULL);
	strftime(TimeString, 80, "%d-%m-%y %H:%M:%S", localtime(&now.tv_sec));
	
	// Build the LOG File data entry+--
	logline[0] = 0;
	sprintf(logline, "%s: %s\r\n", TimeString, comment);	// get header line text
	BHLOG(LOGBH) printf("  BeeLog: %s", logline);
	
	// now write logdata to file
	sprintf(logpath, "%s/%s", cfgini->web_root, cfgini->biot_LOGFILE); 
	bhlog = fopen(logpath, "a");
	
	if (!bhlog) {
		//----- FILE NOT FOUND -----	
		if(!(bhlog = fopen(logpath, "rw"))){	// retry open file with rw parameters
			printf("  BeeLog: File %s not found and could not be created\n", logpath);
			return(-1);
		}
		// else new file created
	}

	fseek(bhlog, 0L, SEEK_END);		// check length of file, and go to end of file 	
	fwrite(&logline[0],sizeof(unsigned char), strlen(logline), bhlog) ;
	fflush(bhlog);			// write and flash file
	fclose(bhlog);

	return(0);	
}

/*******************************************************************************
* Function: beecsv()
* Creation: 22.01.2017
* 
* Write beehive data (all 1-wire sensors + weight scale vale) in a special 
* line format with time stamp into CSV file.
* CSVfile path and name as defined in config.ini
*
* Input :
*	dataset * data	pointer to all log data DB
*
* Global:	
*	cfgini struct	struct of runtime parameters
*	beelog_<date>.csv file
*
* Output:
*	return		 0 log data written to LOG file, 
*				-1 file could not be found/opened
*
*********************************************************************************
*/
int beecsv(dataset* data){

char	logline[LOGLINELEN];	// text buffer of one log line
char	sbuf[LOGLINELEN];		// univ. string buffer
FILE *	bhlog;					// file handle of logfile
char	logpath[1024];			// log file path buffer	
int		i,len;
struct timeval	now;
int		idx;	
	
	idx = 0;		// get curr. data row index
	gettimeofday(&now, NULL);
		
	// Build the CSV File data entry
	logline[0] = 0;

// get log line text based on CSV_HEADER format !
	sprintf(logline, "%s,%3.3f,%2.2f,%2.2f,%2.2f,%2.2f,%2.2f,%2.2f,%2.2f,%2.2f,%i #%i %s\r\n", 
				data->dlog[idx].timeStamp, 
				data->dlog[idx].HiveWeight,
				data->dlog[idx].TempExtern,
				data->dlog[idx].TempIntern,
				data->dlog[idx].TempHive,
				data->dlog[idx].TempRTC,
				data->dlog[idx].ESP3V,
				data->dlog[idx].Board5V,
				data->dlog[idx].BattCharge,
				data->dlog[idx].BattLoad,
				data->dlog[idx].BattLevel,
				data->dlog[idx].index,
				data->dlog[idx].comment);	// with finalized EOL for Windows & Linux

//	strncpy(logline, data->status, LOGLINELEN);
//	strncat(logline, "\r\n", 2);

	// Force CSV Date format with '/'
	if(logline[4] != '/'){	// Correct wrong date separator to YYYY/MM/DD HH:MM:SS
		logline[4] = '/';
		logline[7] = '/';		
	}

	// now write logdata to log file
	// first, get timestamp of curr. log data -> year only	
	//	strftime(TimeString, 80, "%Y", localtime(&data->dlog[idx].tvsec));
	strncpy(TimeString, bhdb.date, 4);	// get year string only
	TimeString[4]=0;
	sprintf(logpath, "%s/%s%02X_%s.csv", cfgini->web_root, cfgini->biot_CSVFILE, data->NodeID, TimeString); 

	// do we have already a log file existing
	bhlog = fopen(logpath, "a");
	
	if (!bhlog) {
		// original log file not existing -----> FILE NOT FOUND -----	
		if(!(bhlog = fopen(logpath, "rw"))){	// retry open file with rw parameters
			printf("  BeeCSV: File not found and could not be created at %s\n", logpath);
			return(-1);
		}
		// else new file created
		sprintf(sbuf, "    BeeCSV: New %s file created", logpath);
		beelog(sbuf);
	}

	fseek(bhlog, 0L, SEEK_END);		// check length of file 
	if (ftell(bhlog) == 0){			// -> =0 new file created: needs headerline first
		BHLOG(LOGBH) printf("    BeeCSV: New LogFile created\n");
		fseek(bhlog, 0L, SEEK_SET);				// go back to start of file
		sprintf(sbuf, "%s\n", CSV_HEADER);	// get header line text
		fwrite(&sbuf[0],sizeof(unsigned char), strlen(sbuf), bhlog) ;
		fflush(bhlog);			// write and flash file
	}

	fwrite(&logline[0],sizeof(unsigned char), strlen(logline), bhlog) ;

	fflush(bhlog);			// write and flash file
	fclose(bhlog);
	printf("    BeeCSV: %s> %s", logpath, logline);

	return(0);	
}
