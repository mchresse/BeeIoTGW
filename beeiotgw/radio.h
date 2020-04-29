//*******************************************************************
// File:	radio.h  
// Project: https://github.com/mchresse/BeeIoTGW
// Author:	MCHREsse
// Created on 11. April 2020, 15:28
//
// Description:
// BeeIoT-GateWay - BIoTWAN LoRa Radio class definition
// + global Modem & channel data structs: modemcfg_t + chncfg_t
//
//----------------------------------------------------------
// Copyright (c) 2020-present, Randolph Esser
// All rights reserved.
// This file is distributed under the BSD-3-Clause License
// The complete license agreement can be obtained at: 
//     https://github.com/mchresse/BeeIoTGW/license
// For used 3rd party open source see also Readme_OpenSource.txt
//*******************************************************************
#ifndef _RADIO_H
#define _RADIO_H

#include "regslora.h"

//***************************************************************
// LoRa Modem Hardware IO & IRQ callback function declarations
class Radio;	// forward declaration for typedef
typedef void (Radio::*IrqHandler )( void );	// prototype of DIOx-IRQ handler routines

// Global modem HW IO configuration settings (per instance)
typedef struct {
		byte	sxcs;		// Chip Select
		byte	sxrst;		// Reset line
		byte	sxdio0;		// DIOx IRQ line
		byte	sxdio1;		// DIOx IRQ line
		byte	sxdio2;		// DIOx IRQ line
		byte	sxdio3;		// DIOx IRQ line
		byte	sxdio4;		// DIOx IRQ line		
		byte	sxdio5;		// DIOx IRQ line		
}iopins_t;

// Global Modem descriptor
typedef struct{
	byte		modemid;	// index of modem instance	-> set by main() cfgini
	byte		gwid;		// modem corresponding Pkg GWIDx
	byte		chncfgid;	// ID of channel configuration set
	iopins_t	iopins;		// GPIO port definition	-> set by main() cfgini
	Radio *		modem;		// ptr to modem instance -> set by constructor
	MsgQueue *	gwq;		// ptr to modem Msg Queue for all GW channels

    // Statistic: to be initialize/updated by radio layer during rx/tx package handling
    uint32_t cp_nb_rx_rcv;	// # received packages
    uint32_t cp_nb_rx_ok;	// # of correct received packages
    uint32_t cp_up_pkt_fwd;	// # of sent status packages to REST/WEb service
    uint32_t cp_nb_rx_bad;	// # of invalid RX packages
    uint32_t cp_nb_rx_crc;	// # of RX packages /w CRC error
}modemcfg_t;

// Radio-Modem internal used config channel set
typedef struct{ 
	long freq;				// =EU868_F1..9,DN (EU868_F1: 868.1MHz)
	s1_t pw;				// =2-16  TX PA Mode (14)
	sf_t sf;				// =0..8 Spreading factor FSK,7..12,SFrFu (1:SF7)
	bw_t bw;				// =0..3 RFU Bandwidth 125-500 (0:125kHz)
	cr_t cr;				// =0..3 Coding mode 4/5..4/8 (0:4/5)
	byte ih;				// =1 implicite Header Mode (0)
	u1_t ihlen;				// =0..n if IH -> header length (0)
	u1_t nocrc;				// =0/1 no CRC check used for Pkg (0)
	u1_t noRXIQinv;			// =0/1 flag to switch RX+TX IQinv. on/off (1)
}chncfg_t;

// Radio internal housekeeping
typedef struct {
	byte	 modemid;		// Modem ID: 0.. cfgini->loranumchn;
	chncfg_t chncfg;		// Configuration of modem LoRa transmit channel

	// current LoRa Modem Op.Mode
	byte currentMode;		// Modem status: SLEEP/IDLE/RX/TX
	byte chiptype;			// Modem chip type: Semtech LoRa Chip: SX1276 0x12, =0: unknown
	MsgQueue * gwq;				// local Ptr on Modem MsgQueue of LoRa Packages
	
	// used by ISR routine
	int	 irqlevel;			// =0..n IRQ Enable semaphore: (0: IRQs allowed) 
	byte snr;				// Signal/Noise Ratio level [db]
	byte rssi;				// Received signal strength indication [dbm]
	byte rxbuffer[256];		// generic frame buffer for unknown packages
	byte rxdlen;			// length of generic/unknown package
	byte txbuffer[256];		// universal TX buffer
	byte txdlen;			// length of TX package to be sent

	// (initialized by radio_init(), used by radio_rand1())
	u1_t randbuf[16];

	struct timeval now;		// current timestamp used each time a ISR time check is done
	unsigned long txstart;	// tstamp when TX Mode was entered
	unsigned long txend;	// Delta: now - txstart
	unsigned long rxtime;	// tstamp when last rx package arrived
} modemset_t;

//*****************************************
// enhanced version for diff. power classes
enum policy_t {
	LMICHAL_radio_tx_power_policy_20dBm, 
	LMICHAL_radio_tx_power_policy_paboost,
	LMICHAL_radio_tx_power_policy_rfo
};



//******************************************************************************
// Lora Radio Layer class declaration
// Handling of Semtech SX1276 chip communication and IRQ service
// RD/WR packages via Queue member functions

class Radio {
//***************************************************************
public:
//******************************
// RADIO member function / API:

	// RADIO Constructor: 
	// default is modem index=0, ín Multi modem mode: used for JOIN requests only
	Radio(modemcfg_t * modemcfg); // Detect SX lora chip and setup config channel
	~Radio();					// reset SX Chip (Sleep)

	int	 getmodemidx(void);		// deliver index of modem instance
	long getchannelfrq(void);	// channel frequency e.g.868.1MHz
	sf_t getspreading(void);	// channel spreading factor SFx
	bw_t getband(void);			// Bandwidth: 
	cr_t getcoding(void);		// coding rate
	int	 getirqlevel(void);		// irqlevel: =0 IRQ enabled
	s1_t getpower(void);		// TX power rate 1..21
	
	// print LoraRegister Status to Serial port in diff. sizes
	void PrintLoraStatus(int logtype);

	// Deliver current Modem OpMode
	byte getopmode(void);

	// TX pkg frame via Lora Modem
	int  starttx (byte* frame, byte dataLen, bool async);
	// RX Lora Modem Channel in RXSingle or RXCont Mode
	void startrx (u1_t rxmode, int rxtime);
	
	// link GW Queue to modem session async to instance creation
	void assign_gwqueue(MsgQueue & gwq);

	// Helper function from MsgBuffer: Direct SX FiFo read to MsgBuffer
	friend int	MsgBuffer::setpkgfifo(Radio * Modem, byte sxreg, int dlen);

	// Attach (non-member !) ISR function ptr (created as Lambda function) to IRQ GPIO line
	void Radio_AttachIRQ(uint8_t irq_pin, int irqtype, void (*ISR_callback)(void));
	void MyIRQ0(void);		// DIO0 IRQ member function wrapper
	void MyIRQ1(void);		// DIO1 IRQ member function wrapper
	void MyIRQ2(void);		// DIO2 IRQ member function wrapper

	// MyIRQx ISR function ptr table for dyn. GPIO-DIOx assignment used in isr_init();
	IrqHandler *dioISR;		// Ptr table root allocated by Radio()

		//******************************************************************************
protected:

	
//******************************************************************************
private:
	
	//**************************************************************************
	// RADIO data: default cfg. struct
	//
	// *ModemP: ptr to global modem core configuration table defined in main()-init_all()
	modemcfg_t* modemp;		// major static input for Radio constructor
	
	// internal modem channel configuration settings for runtime usage
	modemset_t  mset;       // housekeeping: status and user->init values    	
    
	// TX Done Flag: =1 TXDone / channel free, =0 TX still in work
    bool        TXDoneFlag;	// Semaphore for sent message finalization,set by ISR

	//**************************************************************************
	// RADIO member functions: 
	// - HAL functions for Reset / RD / WR via SPI bus
	//   using WiringPi lib
	void hal_pin_nss (u1_t val);		// Control CS line of SX chip
	
	// RADIO HAL: Core SPI write of given byte out to radio, read byte from radio and return value.
	u1_t hal_spi (u1_t out);			// send 1 Byte via SPI channel via wiringPi Lib	

	// RADIO HAL: Core SPI transfer with address and one single byte:
	u1_t hal_spi_single (u1_t address, u1_t out); 

	// RADIO HAL: SPI cycle: Enable CS, set address and write 1 Byte, disable CS
	void writeReg (u1_t addr, u1_t data );

	// RADIO HAL: SPI cycle: Enable CS, set address and read 1 Byte, disable CS
	u1_t readReg (u1_t addr);

	// RADIO HAL: SPI cycle: Enable CS, Loop(len): set address and write bytewise from buffer, disable CS
	void writeBuf (u1_t addr, xref2u1_t buf, u1_t len);

	// RADIO HAL: SPI cycle: Enable CS, Loop(len): set address and read bytewise to buffer, disable CS
	void readBuf (u1_t addr, xref2u1_t buf, u1_t len);

	// RADIO HAL: Reset RFM96 module == SX1276 base
	void resetModem(void);
	
	
	// Get Modem Channel configuration by cfgidx ==> index for mset.chncfg[]
	void setchannelconfig(byte cfgidx);

	//Detect supported modem chiptype
	int	 getchiptype(void);

	// Initialize operation mode of SX chip: LORA + SLEEP
	void setLoraMode(void);
	
	// Set SX chip Operation mode
	void setopmode (u1_t mode);

	// set Lora Modem channel frequency
	void configChannelFreq(void);

	// Configure Lora Modem Registers cfg1+2+3 based on config channel settings
	void configLoraModem(void);	
	
	// get random seed from wideband noise rssi
	void radio_init(void);
	
	// preset HW GPIO definitions for selected Modem of Radio-Instance
//	int  setmodemcfg(byte channelidx);

	// Set TX Power configuration
	void configPower (void);	// easy way	(deprecated)
	void configPower2 (void);	// by LMIC: negotiate power class policy by bandwidth

	// return next random byte derived from randbuf seed buffer
	// (buf[0] holds index of next byte to be returned)
	u1_t radio_rand1(void);	// read from randbuf + AES encoded
	byte LoRa_random(void);	// simple random value from Read RSSI-Wideband register
	u1_t radio_rssi (void);	// raw read of RSSI Reg Value - IRQ save

	// TX Lora Frame
	void txlora ( byte* frame, byte dataLen);

	// Read Lora Frame in Single or RXCont Mode 
	void rxlora (u1_t rxmode, int rxto);	

	// Assign DIOx-IRQ wrapper to DIOx lane
	void(*ISR_callback)();
	void myradio_irq_handler (byte dio);
	void hal_disableIRQs(void);
	void hal_enableIRQs(void);

}; // end of RADIO-class

#endif /* _RADIO_H */