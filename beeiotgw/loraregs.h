/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   loraregs.h
 * Author: MCHREsse
 *
 * Created on 18. Dezember 2019, 12:44
 */

#ifndef LORAREGS_H
#define LORAREGS_H

// #############################################
// RFM96W SPI register definitions as by SX1276
// #############################################
// Cfg. regs to be written only in Sleep or Standby mode !

#define REG_VERSION	  				0x42

// ----------------------------------------
// values for REG_OPMODE in LoRA Selection (+0x80)!
#define REG_OPMODE                  0x01	// use only LoraMode (0x80) + OP Mode

// Masks for REG_OPMODE:
#define LF_MODE_ON		0x08
#define LORA_MODE_ON	0x80
#define OM_MASK			0x07
#define OM_LORA			0x80
#define OM_LORA_MASK	OM_MASK | OM_LORA

#define OM_LORA_SLEEP     0x80
#define OM_LORA_STANDBY   0x81
#define OM_LORA_FSTX      0x82
#define OM_LORA_TX        0x83		// single shot TX -> back to standby
#define OM_LORA_FSRX      0x84
#define OM_LORA_RX_CONT	  0x85		// endless read mode
#define OM_LORA_RX_SINGLE 0x86		// single shot RX -> back to standby
#define OM_LORA_CAD       0x87


// ----------------------------------------
// FiFo is cleared in sleep mode; RX&TX field can have ident. base and size (max. 256)
#define REG_FIFO                    0x00	// max. 256 Bytes Buffer for RX & TX usage
#define REG_FIFO_ADDR_PTR           0x0D	// RD & WR Ptr. for SPI access to RX/TX field (auto incr.)
#define REG_FIFO_TX_BASE_AD         0x0E	// Ptr. to Start of FiFo TX Payload field def:0x80
#define REG_FIFO_RX_BASE_AD         0x0F	// Ptr. to Start of FiFo RX field: def:0x00
#define REG_RX_NB_BYTES             0x13	// # of received bytes
#define REG_FIFO_RX_BYTE_AD			0x25	// Addr. of lastbyte written in FIFO
#define REG_FIFO_RX_CURRENT_ADDR    0x10	// Read Ptr. for LoRa Modem access
                                            // points to start of last received package in RX field
                                            // as new base for FIFO_ADDR_PTR to read it
// ----------------------------------------
#define REG_IRQ_FLAGS_MASK          0x11
#define REG_IRQ_FLAGS               0x12
// Bits masking the corresponding IRQs from the radio
#define IRQ_LORA_RXTOUT_MASK		0x80
#define IRQ_LORA_RXDONE_MASK		0x40
#define IRQ_LORA_CRCERR_MASK		0x20
#define IRQ_LORA_HEADER_MASK		0x10
#define IRQ_LORA_TXDONE_MASK		0x08
#define IRQ_LORA_CDDONE_MASK		0x04
#define IRQ_LORA_FHSSCH_MASK		0x02
#define IRQ_LORA_CDDETD_MASK		0x01

// ----------------------------------------
// DIO function mappings                    D0D1D2D3
#define REG_DIO_MAPPING_1           0x40  // Mapping of pins DIO0 to DIO3
#define REG_DIO_MAPPING_2           0x41  // Mapping of pins DIO4 to DIO5, ClkOutFrequ.
#define MAP_DIO0_LORA_RXDONE		0x00  // 00------	RXDone in SingleRX mode
#define MAP_DIO0_LORA_SYNCADDR		0x00  // 00------	SyncAddr. valid in Cont.RX mode
#define MAP_DIO0_LORA_TXDONE		0x40  // 01------
#define MAP_DIO1_LORA_RXTOUT		0x00  // --00----
#define MAP_DIO1_LORA_NOP			0x30  // --11----
#define MAP_DIO2_LORA_NOP			0x0C  // ----11--
#define MAP_DIO3_LORA_NOP			0x03  // ------11


// ----------------------------------------
#define REG_RX_HEADER_CNT_VALUE_MSB 0x14	// Number of valid Headers received
#define REG_RX_HEADER_CNT_VALUE_LSB 0x15
#define REG_RX_PACKET_CNT_VALUE_MSB 0x16	// Number of valid packages received
#define REG_RX_PACKET_CNT_VALUE_LSB 0x17

#define REG_MODEM_STAT  		    0x18	// Live LoRa Modem Status
#define REG_MODEM_CONFIG1           0x1D
#define REG_MODEM_CONFIG2           0x1E
#define REG_MODEM_CONFIG3           0x26
#define REG_SYMB_TIMEOUT_LSB  		0x1F
#define REG_PKT_SNR_VALUE			0x19
#define REG_PKT_RSSI_VALUE			0x1A
#define REG_RSSI_VALUE  			0x1B	// current RSSI
#define REG_RSSI_WIDEBAND			0x2c
#define REG_HOP_PERIOD              0x24
#define REG_SYNC_WORD				0x39
#define REG_PA_CONFIG				0x09	// PA selection & Power control
#define REG_PA_RAMP					0x0A
#define REG_PA_DAC					0x4D	// Higher Power Settings of the PA
#define REG_PREAMBLE_MSB			0x20
#define REG_PREAMBLE_LSB			0x21
#define REG_PLL_BW					0x70	// PLL Bandwidth setting

#define REG_PAYLOAD_LENGTH          0x22	// # of bytes to be transmitted
#define REG_MAX_PAYLOAD_LENGTH 		0x23	// Max. Payloaf set to 0x80
#define PAYLOAD_LENGTH              0x40	// def. # of bytes to be transmitted
#define MAX_PAYLOAD_LENGTH          0x80	// max. # of bytes to be transmitted

// ----------------------------------------
// LOW NOISE AMPLIFIER
#define REG_LNA                     0x0C
#define LNA_MAX_GAIN                0x23
#define LNA_OFF_GAIN                0x00
#define LNA_LOW_GAIN		    	0x20

// CONF REG values
#define REG1                        0x0A
#define REG2                        0x84

// ----------------------------------------
// sx1276 RegModemConfig1
// bandwidth for RegModemConfig1
#define SX76_MC1_BW_125            0x70
#define SX76_MC1_BW_250            0x80
#define SX76_MC1_BW_500            0x90
// coding rate for RegModemConfig1
#define SX76_MC1_CR_4_5            0x02
#define SX76_MC1_CR_4_6            0x04
#define SX76_MC1_CR_4_7            0x06
#define SX76_MC1_CR_4_8            0x08
#define SX76_MC1_IMPLICIT_HEADER_MODE_ON    0x01 // required for receive

// sx1276 RegModemConfig2
#define SX76_MC2_RX_PAYLOAD_CRCON        0x04
#define SX76_MC2_SF6                0x60
#define SX76_MC2_SF7                0x70
#define SX76_MC2_SF8                0x80
#define SX76_MC2_SF9                0x90
#define SX76_MC2_SF10               0xA0
#define SX76_MC2_SF11               0xB0
#define SX76_MC2_SF12               0xC0

// sx1276 RegModemConfig3
#define SX76_MC3_LOW_DATA_RATE_OPTIMIZE  0x08
#define SX76_MC3_AGCAUTO                 0x04

// Sync word for Private LoRa networks
#define LORA_MAC_PRIVATE_SYNCWORD        0x12
// Sync word for Public LoRa networks
#define LORA_MAC_PUBLIC_SYNCWORD         0x34
// preamble for lora networks (nibbles swapped)
#define LORA_MAC_PREAMBLE                LORA_MAC_PUBLIC_SYNCWORD

#define RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG1 0x0A
#define RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG2 0x70		// for SX1276

/*!
 * SX1276 definitions
 */
#define XTAL_FREQ                   32000000
#define FREQ_STEP                   61.03515625

#define RX_BUFFER_SIZE              255

/*!
 * Constant values need to compute the RSSI value
 */
#define RSSI_OFFSET_LF              -164.0
#define RSSI_OFFSET_HF              -157.0

#define RF_MID_BAND_THRESH          525000000
// ----------------------------------------
// Set FRF register
#define REG_FRF_MSB			0x06
#define REG_FRF_MID			0x07
#define REG_FRF_LSB			0x08

// FRF value for 868.1MHz
#define FRF_MSB				0xD9
#define FRF_MID				0x06
#define FRF_LSB				0x66

// ----------------------------------------
#define PROTOCOL_VERSION	1
#define PKT_PUSH_DATA		0
#define PKT_PUSH_ACK		1
#define PKT_PULL_DATA		2
#define PKT_PULL_RESP		3
#define PKT_PULL_ACK		4

#define TX_BUFF_SIZE		2048
#define STATUS_SIZE			1024
#define BUFLEN				2048			//Max length of buffer


#endif /* LORAREGS_H */
