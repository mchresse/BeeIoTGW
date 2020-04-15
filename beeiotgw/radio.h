/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   radio.h
 * Author: mchresse
 *
 * Created on 11. April 2020, 15:28
 */

#ifndef _RADIO_H
#define _RADIO_H

#include "regslora.h"


	
//******************************************************************************
// Lora Radio Layer class
// Handling of Semtech SX1276 chip communication and IRQ service
// RD/WR packages via Queue member functions

class Radio {
//******************************************************************************
public:
	byte	sxcs, sxrst, sxdio0, sxdio1, sxdio2, sxdio3, sxdio4, sxdio5;

	//*****************************************************************************
	// RADIO member function / API
	// RADIO Constructor: default is cfg channel index=0 (for JOIN requests)
	Radio(byte channelidx=0);	// -> SetupLora() => discover SX chip and setup config channel by idx
	~Radio();	// release current SX Chip an SPI channel

	int	 GetModemIdx(void);		// deliver index of modem instance
	long getchannelfrq(void);	// channel frequency e.g.868.1MHz
	sf_t getspreading(void);	// channel spreading factor SFx
	bw_t getband(void);			// Bandwidth: 
	cr_t getcoding(void);		// coding rate
	int	 getirqlevel(void);		// irqlevel: =0 IRQ enabled
	s1_t getpower(void);		// TX power rate 1..21
	
	// print LoraRegister Status to Serial port in diff. sizes
	void PrintLoraStatus(int logtype);

	// Deliver cuurent Modem OpMode
	byte getopmode(void);
	// Set SX chip Operation mode
	void setopmode (u1_t mode);

	// Set Modem Channel configuration: cfgidx == index to txchntab[]
	void setchannelconfig(byte cfgidx);

	// TX Frame via Lora Modem
	void starttx (byte* frame, byte dataLen);
	// RX Lora Modem Channel in Single or RXCont Mode
	void startrx (u1_t rxmode, int rxtime);
	
	void Radio_AttachIRQ(uint8_t irq_pin, int irqtype, void (*ISR_callback)(void));
	void MyIRQ0(void);	// DIO0 IRQ wrapper
	void MyIRQ1(void);	// DIO1 IRQ wrapper
	void MyIRQ2(void);	// DIO2 IRQ wrapper
		
//******************************************************************************
protected:
	// (initialized by radio_init(), used by radio_rand1())
	u1_t randbuf[16];

	//*****************************************************************************
	// RADIO layer: default cfg settings
	//*****************************************************************************
	long freq	= FREQ;					// =EU868_F1..9,DN (EU868_F1: 868.1MHz)
	s1_t pw		= TXPOWER;				// =2-16  TX PA Mode (14)
	sf_t sf		= SPREADING;			// =0..8 Spreading factor FSK,7..12,SFrFu (1:SF7)
	bw_t bw		= SIGNALBW;				// =0..3 RFU Bandwidth 125-500 (0:125kHz)
	cr_t cr		= CODING;				// =0..3 Coding mode 4/5..4/8 (0:4/5)
	byte ih		= IHDMODE;				// =1 implicite Header Mode (0)
	u1_t ihlen	= IHEADERLEN;			// =0..n if IH -> header length (0)
	u1_t nocrc	= NOCRC;				// =0/1 no CRC check used for Pkg (0)
	u1_t noRXIQinversion = NORXIQINV;	// =0/1 flag to switch RX+TX IQinv. on/off (1)
	
	// current LoRa Modem Op.Mode
	byte	currentMode = OPMODE_LORA | OPMODE_SLEEP; // (start LoRa Modem in SLEEP Mode)
	bool	sx1276;				// Semtech LoRa ChipType: =0(SX1276), =1(SX1272) (0)
	byte	modem=0;			// Modem ID: 0.. cfgini->loranumchn;
	
	// used by ISR routine
	int	 irqlevel = 0;					// =0..n IRQ Enable semaphor: (0: IRQs allowed) 
	byte snr;					// Signal/Noise Ratio level
	byte rssi;					// 
	byte rxframe[256];			// generic frame buffer for unknown packages
	byte rxdlen = 0;			// length of generic/unknown package
	byte txframe[256];			// universal TX buffer
	byte txdlen = 0;			// length of TX package to be sent

	struct timeval now;			// current timestamp used each time a ISR time check is done
	unsigned long txstart;		// tstamp when TX Mode was entered
	unsigned long txend;		// Delta: now - txstart
	unsigned long rxtime;		// tstamp when last rx package arrived

	//*****************************************
	// enhanced version for diff. power classes
	enum policy_t {
		LMICHAL_radio_tx_power_policy_20dBm, 
		LMICHAL_radio_tx_power_policy_paboost,
		LMICHAL_radio_tx_power_policy_rfo
	};

	// preset HW GPIO definitions for selected Modem of Radio-Instance
	int  setmodemcfg(byte channelidx);
	
	// RADIO HAL member functions using WiringPi lib
	void hal_pin_nss (u1_t val);		// Control CS line of SX chip
	
	// Core SPI write of given byte out to radio, read byte from radio and return value.
	u1_t hal_spi (u1_t out);			// send 1 Byte via SPI channel via wiringPi Lib	

	// Core SPI transfer with address and one single byte:
	u1_t hal_spi_single (u1_t address, u1_t out); 

	// Reset RFM96 module == SX1276 base
	void resetModem(void);

	// SPI cycle: Enable CS, set address and write 1 Byte, disable CS
	void writeReg (u1_t addr, u1_t data );

	// SPI cycle: Enable CS, set address and read 1 Byte, disable CS
	u1_t readReg (u1_t addr);

	// SPI cycle: Enable CS, Loop(len): set address and write bytewise from buffer, disable CS
	void writeBuf (u1_t addr, xref2u1_t buf, u1_t len);

	// SPI cycle: Enable CS, Loop(len): set address and read bytewise to buffer, disable CS
	void readBuf (u1_t addr, xref2u1_t buf, u1_t len);

	// Set SX chip Operation mode
	void opmode (u1_t mode);
	
	// Initialize operation mode of SX chip: LORA + SLEEP
	void opmodeLora(void);
	
	// Configure Lora Modem Registers cfg1+2+3 based on config channel settings
	void configLoraModem(void);	
	
	// set Lora Modem Config channel: FREQ
	void configChannel(void);

	// Set TX Power configuration
	void configPower (void);	// easy way	(deprecated)
	void configPower2 (void);	// by LMIC: negotiate power class policy by bandwidth
	
	// get random seed from wideband noise rssi
	void radio_init(void);

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

