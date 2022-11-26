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


int beecmd_parse(cmd_t * cmd, char cmdline[], int ndid);

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


/*******************************************************************************
* Function: beecmd()
* Creation: 02.01.2022
* 
* Read BEE-CMD File for special command send in a next free RX1 window
* If result data is expected -> written to BEE-Result file.
*
* Input :
*	ndid	NodeID of current cmd session -> for filtering the right command line
*   *cmd	CMD-struct: special command from BIOTWAN.h command file
* Global:	
*	cfgini struct	struct of runtime parameters
*	Bee-CMD File: Expecting 1 Command line with format:
*		<NodeID#>: <Cmd>	optional: <Par1> <Par2>
*
* Output:
*	return		0	Bee-Cmd file was empty 
*				1	CMD-struct code updated
*********************************************************************************
*/

int beecmd(cmd_t * cmd, int ndid){
char	cmdline[LOGLINELEN];	// text buffer of command string
FILE *	bhcmd;					// file handle of cmdfile
char	filepath[1024];			// file path buffer	
int		rc;

	cmd->cmdcode = CMD_NOP;	// default cmd code: do nothing
	cmd->p1 = -1;			// mark Param as empty/n.a.
	cmd->p2 = -1;			// mark Param as empty/n.a.
	cmd->p3 = -1;			// mark Param as empty/n.a.	

	// now read command line from file
	sprintf(filepath, "%s/%s", cfgini->web_root, cfgini->biot_CMDFILE); 
		//----- FILE NOT FOUND -----	
	
	bhcmd = fopen(filepath, "r+");
	if(bhcmd == NULL){	// no cmd file found
		BHLOG(LOGBH)printf("  BeeCmd: File %s not found\n", filepath);
		return(-1);
	}

	fseek(bhcmd, 0L, SEEK_END);		// check length of file
	if(ftell(bhcmd) == 0){
		// BHLOG(LOGBH)printf("  BeeCmd: CMD-File at %s empty\n", filepath);
		fclose(bhcmd);
		return(0);
	}
	// Get 1. (and only !) command line
	fseek(bhcmd, 0L, SEEK_SET);		// return back to start of file

	rc = -1;	// preset to "no cmd line found"
	do{
		if(fgets(cmdline, sizeof(cmdline)-2, bhcmd) != NULL){ // read 1 line inclusive EOL + /0
			rc = beecmd_parse(cmd, cmdline, ndid); // Parse retrieved command line
		}else{	//EOL reached /wo any valid line
			BHLOG(LOGBH)printf("  BeeCmd: No valid CMD string: %s in %s found\n", cmdline, filepath);		
			fclose(bhcmd);
			return(0);
		}
	} while(rc < 1);	// read next cmdline and skip comment lines with leading '#'

	return(1);	
}

//******************************************************************************
// BeeCmd()
// Input:	cmd		 struct of cmd line + fields for parsed cmd + params
//	        cmdline	 1 command line string from Bee-Cmd file
// Return:  =0	Comment line only or wrong ndid -> skip
//			=1  Command line parsed successfully
//			=-1 Command line wrong/unknown format
//			=-2 Unsupported Command code
//******************************************************************************
int beecmd_parse(cmd_t * cmd, char * cmdline, int ndid){	
#define CMDSTRLEN 16
char	cmdstr[CMDSTRLEN];
int		id =0;		// cmdline Node-ID: 0 ... 9 (MAXNODES-1)
int		rc;
const char * format ="%d: %s %d %d %d";

	if(cmdline[0] == '#'){	// skip/ignore comment lines
		return(0);
	}

	rc = sscanf( cmdline, format, &id, cmdstr, &cmd->p1, &cmd->p2, &cmd->p3 );   // Cmd + params
	
	// check range of valid # of params + cmd-length + NodeID range
	if(id != ndid) {	// skip wrong command line assignment to nodeid	
		return(0);
	}
	if(	(rc < 2) || (rc > 5) ||	// min 2, max 4 params accepted: id + cmd + p1 + p2
		(strlen(cmdstr)!= 2)){	// only 2 letter commands accepted
		// skip wrong command line assignment to nodeid	
	//	BHLOG(LOGBH)printf("  BeeCmd: Invalid CMD string format(%d): %s\n", rc, cmdline);
		return(-1);
	}
	
	// Valid command line of current node detected: Parse command string and return Command code

	// GETSDDIR: Get SD card directory Structure
	if( strncmp((char *) &cmdstr, "SX", 2)==0 ||  strncmp((char *)&cmdstr, "sx", 2)==0){
		BHLOG(LOGBH)printf("  BeeCmd: RX1-CMD: %s  Params: %d %d %d\n", cmdstr, cmd->p1, cmd->p2, cmd->p3);
		cmd->cmdcode = CMD_GETSDDIR;
		return(1);
	}
	// GETSDDATA: Get SD Card data chunk
	if( strncmp((char *) &cmdstr, "SD", 2)==0 ||  strncmp((char *)&cmdstr, "sd", 2)==0){
		BHLOG(LOGBH)printf("  BeeCmd: RX1-CMD: %s  Params: %d %d %d\n", cmdstr, cmd->p1, cmd->p2, cmd->p3);
		cmd->cmdcode = CMD_GETSDLOG;
		return(1);
	}
	// RESET: Reset Node Statistics, JOIN mode, Job Queue
	if( strncmp((char *) &cmdstr, "RS", 2)==0 ||  strncmp((char *)&cmdstr, "rs", 2)==0){
		BHLOG(LOGBH)printf("  BeeCmd: RX1-CMD: %s  Params: %d %d %d\n", cmdstr, cmd->p1, cmd->p2, cmd->p3);
		cmd->cmdcode = CMD_RESET;
		char resbuf[100]="Node Reset initiated";
		beeResultlog((char*)&resbuf[0], ndid, 1);
		
		return(1);
	}
	
	return(-2);	// unsupported command code detected
}

/*******************************************************************************
* Function: beeResultlog()
* Creation: 08.01.2022
* 
* Write BEE-Result File with received message string from RX1 command stream
*
* Input :
*   result	Ptr. on RX1 result stream line 
*	ndid	NodeID of current cmd session -> for filtering the right command line
*
* Global:	
*	cfgini struct	struct of runtime parameters
*	Bee-Result File: Writing 1 Result line with format:
*		<NodeID#>: <Index:> <result string>
*
* Output:
*	return		0	Bee-Result file was empty 
*********************************************************************************
*/
int beeResultlog(char * result, int ndid, uint8_t seqid){
FILE *	bhresult;				// file handle of resultfile
FILE *	bhcmd;					// file handle of cmdfile
char	filepath[1024];			// file path buffer	
char	resultstr[256];
int		rc;

	// now write stream data to Bee result file
	sprintf(filepath, "%s/%s", cfgini->web_root, cfgini->biot_RESULTFILE); 
	
	bhresult = fopen(filepath, "a");	// append new stream line
	if(bhresult == NULL){	// no result file found
		BHLOG(LOGBH)printf("  BeeResult: File %s not found -> create new one\n", filepath);
		bhresult = fopen(filepath, "rw");	// append new stream line
		if(bhresult == NULL){	// still no result file found
			BHLOG(LOGBH)printf("  BeeResult: File %s not found\n", filepath);
			return(-1);	// give up
		}
	}
	snprintf(resultstr, (size_t)254, "%d:%5d: %s\n\0", ndid, seqid, result);
	// Now Write 1. result line
	fwrite(resultstr, sizeof(unsigned char), strlen(resultstr), bhresult);
	fclose(bhresult);

	// now we have a potential result line written -> cmd file to be deleted
	// to avoid reading same command at next loop again
	sprintf(filepath, "%s/%s", cfgini->web_root, cfgini->biot_CMDFILE);
	bhcmd = fopen(filepath, "w"); // empty cmd file content -> has been discovered now
    fclose(bhcmd);
	// ToDO : delete exactly identified line -> optional cmds  for other nodes in parallel 

	return(0);
}