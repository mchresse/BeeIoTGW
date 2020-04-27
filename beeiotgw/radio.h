/* 
 * File:   radio.h
 * Author: mchresse - Randolph Esser
 *
 * Created on 11. April 2020, 15:28
 */

#ifndef _RADIO_H
#define _RADIO_H

#include "regslora.h"
#include <memory>

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
	byte		chncfg;		// ID of channel configuration set
	iopins_t	iopins;		// GPIO port definition	-> set by main() cfgini
	Radio *		modem;		// ptr to modem instance -> set by constructor
	MsgQueue *	gwq;		// ptr to modem Msg Queue for all GW channels
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
	byte	 modemid;			// Modem ID: 0.. cfgini->loranumchn;
	chncfg_t chncfg;			// Configuration of modem LoRa transmit channel

	// current LoRa Modem Op.Mode
	byte currentMode;			// Modem status: SLEEP/IDLE/RX/TX
	byte chiptype;				// Modem chip type: Semtech LoRa Chip: SX1276 0x12, =0: unknown
	MsgQueue * gwq;				// local Ptr on Modem MsgQueue of LoRa Packages
	
	// used by ISR routine
	int	 irqlevel;				// =0..n IRQ Enable semaphore: (0: IRQs allowed) 
	byte snr;					// Signal/Noise Ratio level [db]
	byte rssi;					// Received signal strength indication [dbm]
	byte rxbuffer[256];			// generic frame buffer for unknown packages
	byte rxdlen;				// length of generic/unknown package
	byte txbuffer[256];			// universal TX buffer
	byte txdlen;				// length of TX package to be sent

	// (initialized by radio_init(), used by radio_rand1())
	u1_t randbuf[16];

	struct timeval now;			// current timestamp used each time a ISR time check is done
	unsigned long txstart;		// tstamp when TX Mode was entered
	unsigned long txend;		// Delta: now - txstart
	unsigned long rxtime;		// tstamp when last rx package arrived
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

	int	 GetModemIdx(void);		// deliver index of modem instance
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

		
	// Hardware DIOx-IRQ function ptr for dyn. table used in  isr_init();
    IrqHandler *dioISR;
	
	// link GW Queue to modem session
	void assign_gwqueue(MsgQueue & gwq);

	// Helper function in Msg Queue handling for direct SX FiFo read to MsgBuffer
	friend int	MsgBuffer::setpkgfifo(Radio * Modem, byte sxreg, int dlen);

	//******************************************************************************
protected:

	
//******************************************************************************
private:


	//**************************************************************************
	// RADIO layer: default cfg. settings
	//
	modemcfg_t* modemp;		// modem related initial config settings for constructor
	modemset_t	mset;		// internal modem channel configuration settings	


	//Detect supported modem chiptype
	int getchiptype(void);

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

