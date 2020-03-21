//*******************************************************************
// regslora.h  
// from Project https://github.com/mchresse/BeeIoT
//
// Description:
// Lora Modem registers of Radio Layer
//
//----------------------------------------------------------
// Copyright (c) 2019-present, Randolph Esser
// All rights reserved.
// This file is distributed under the BSD-3-Clause License
// The complete license agreement can be obtained at: 
//     https://github.com/mchresse/BeeIoT/license
// For used 3rd party open source see also Readme_OpenSource.txt
//*******************************************************************
// BSD 3-Clause License
// 
// BeeIoT Weight Scale project using an own BeeIoTWAN based on LoRa Radio functions
// using a SX1276 LoRa modem chip (e.g. Dragino LoRa/GPS HAT) connected to a 
// Raspberry Pi platform.
//	
// Copyright (c) 2019, Randolph Esser
// All rights reserved.
// 
// This file provides incorporated routines from hal.c + hal.h layer of the project:
//	Hardware Abstraction Layer (HAL) targeted to Raspberry Pi and 
//	Dragino LoRa/GPS HAT
// implemented by
// 
// Copyright (c) 2017, Wolfgang Klenk
// All rights reserved.
/*
 * This file provides incorporated routines out of the great LMIC project 
 *		from radio.c + lmic.h files 
 * as implemented by
 * Copyright (c) 2014-2016 IBM Corporation.
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of the <organization> nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef REGSLORA_H
#define REGSLORA_H

// ---------------------------------------- 
// Registers Mapping
#define RegFifo                                    0x00 // common
#define RegOpMode                                  0x01 // common
#define FSKRegBitrateMsb                           0x02
#define FSKRegBitrateLsb                           0x03
#define FSKRegFdevMsb                              0x04
#define FSKRegFdevLsb                              0x05
#define RegFrfMsb                                  0x06 // common
#define RegFrfMid                                  0x07 // common
#define RegFrfLsb                                  0x08 // common
#define RegPaConfig                                0x09 // common
#define RegPaRamp                                  0x0A // common
#define RegOcp                                     0x0B // common
#define RegLna                                     0x0C // common
#define FSKRegRxConfig                             0x0D
#define LORARegFifoAddrPtr                         0x0D
#define FSKRegRssiConfig                           0x0E
#define LORARegFifoTxBaseAddr                      0x0E
#define FSKRegRssiCollision                        0x0F
#define LORARegFifoRxBaseAddr                      0x0F 
#define FSKRegRssiThresh                           0x10
#define LORARegFifoRxCurrentAddr                   0x10
#define FSKRegRssiValue                            0x11
#define LORARegIrqFlagsMask                        0x11 
#define FSKRegRxBw                                 0x12
#define LORARegIrqFlags                            0x12 
#define FSKRegAfcBw                                0x13
#define LORARegRxNbBytes                           0x13 
#define FSKRegOokPeak                              0x14
#define LORARegRxHeaderCntValueMsb                 0x14 
#define FSKRegOokFix                               0x15
#define LORARegRxHeaderCntValueLsb                 0x15 
#define FSKRegOokAvg                               0x16
#define LORARegRxPacketCntValueMsb                 0x16 
#define LORARegRxpacketCntValueLsb                 0x17 
#define LORARegModemStat                           0x18 
#define LORARegPktSnrValue                         0x19 
#define FSKRegAfcFei                               0x1A
#define LORARegPktRssiValue                        0x1A 
#define FSKRegAfcMsb                               0x1B
#define LORARegRssiValue                           0x1B 
#define FSKRegAfcLsb                               0x1C
#define LORARegHopChannel                          0x1C 
#define FSKRegFeiMsb                               0x1D
#define LORARegModemConfig1                        0x1D 
#define FSKRegFeiLsb                               0x1E
#define LORARegModemConfig2                        0x1E 
#define FSKRegPreambleDetect                       0x1F
#define LORARegSymbTimeoutLsb                      0x1F 
#define FSKRegRxTimeout1                           0x20
#define LORARegPreambleMsb                         0x20 
#define FSKRegRxTimeout2                           0x21
#define LORARegPreambleLsb                         0x21 
#define FSKRegRxTimeout3                           0x22
#define LORARegPayloadLength                       0x22 
#define FSKRegRxDelay                              0x23
#define LORARegPayloadMaxLength                    0x23 
#define FSKRegOsc                                  0x24
#define LORARegHopPeriod                           0x24 
#define FSKRegPreambleMsb                          0x25
#define LORARegFifoRxByteAddr                      0x25
#define LORARegModemConfig3                        0x26
#define FSKRegPreambleLsb                          0x26
#define FSKRegSyncConfig                           0x27
#define LORARegFeiMsb                              0x28
#define FSKRegSyncValue1                           0x28
#define LORAFeiMib                                 0x29
#define FSKRegSyncValue2                           0x29
#define LORARegFeiLsb                              0x2A
#define FSKRegSyncValue3                           0x2A
#define FSKRegSyncValue4                           0x2B
#define LORARegRssiWideband                        0x2C
#define FSKRegSyncValue5                           0x2C
#define FSKRegSyncValue6                           0x2D
#define FSKRegSyncValue7                           0x2E
#define FSKRegSyncValue8                           0x2F
#define LORARegIffReq1                             0x2F
#define FSKRegPacketConfig1                        0x30
#define LORARegIffReq2                             0x30
#define FSKRegPacketConfig2                        0x31
#define LORARegDetectOptimize                      0x31
#define FSKRegPayloadLength                        0x32
#define FSKRegNodeAdrs                             0x33
#define LORARegInvertIQ                            0x33
#define FSKRegBroadcastAdrs                        0x34
#define FSKRegFifoThresh                           0x35
#define FSKRegSeqConfig1                           0x36
#define LORARegHighBwOptimize1                     0x36
#define FSKRegSeqConfig2                           0x37
#define LORARegDetectionThreshold                  0x37
#define FSKRegTimerResol                           0x38
#define FSKRegTimer1Coef                           0x39
#define LORARegSyncWord                            0x39
#define FSKRegTimer2Coef                           0x3A
#define LORARegHighBwOptimize2                     0x3A
#define FSKRegImageCal                             0x3B
#define FSKRegTemp                                 0x3C
#define FSKRegLowBat                               0x3D
#define FSKRegIrqFlags1                            0x3E
#define FSKRegIrqFlags2                            0x3F
#define RegDioMapping1                             0x40 // common
#define RegDioMapping2                             0x41 // common
#define RegVersion                                 0x42 // common
// #define RegAgcRef                                  0x43 // common
// #define RegAgcThresh1                              0x44 // common
// #define RegAgcThresh2                              0x45 // common
// #define RegAgcThresh3                              0x46 // common
// #define RegPllHop                                  0x4B // common
// #define RegTcxo                                    0x58 // common
#define RegTcxo                                    0x4B // common       different addresses, same bits
#define RegPaDac                                   0x4D // common       differnet addresses, same bits
// #define RegPll                                     0x5C // common
// #define RegPllLowPn                                0x5E // common
// #define RegFormerTemp                              0x6C // common
#define RegBitRateFrac                             0x70 // common

#define RegTcxo_TcxoInputOn                        (1u << 4)

// ----------------------------------------
// spread factors and mode for RegModemConfig2
#define SX1272_MC2_FSK  			0x00
#define SX1272_MC2_SF7  			0x70
#define SX1272_MC2_SF8  			0x80
#define SX1272_MC2_SF9  			0x90
#define SX1272_MC2_SF10 			0xA0
#define SX1272_MC2_SF11 			0xB0
#define SX1272_MC2_SF12 			0xC0
// bandwidth for RegModemConfig1
#define SX1272_MC1_BW_125			0x00
#define SX1272_MC1_BW_250			0x40
#define SX1272_MC1_BW_500			0x80
// coding rate for RegModemConfig1
#define SX1272_MC1_CR_4_5			0x08
#define SX1272_MC1_CR_4_6			0x10
#define SX1272_MC1_CR_4_7			0x18
#define SX1272_MC1_CR_4_8			0x20
#define SX1272_MC1_IMPLICIT_HEADER_MODE_ON	0x04 // required for receive
#define SX1272_MC1_RX_PAYLOAD_CRCON		0x02
#define SX1272_MC1_LOW_DATA_RATE_OPTIMIZE	0x01 // mandated for SF11 and SF12
// transmit power configuration for RegPaConfig
#define SX1272_PAC_PA_SELECT_PA_BOOST		0x80
#define SX1272_PAC_PA_SELECT_RFIO_PIN		0x00


// sx1276 RegModemConfig1
#define SX1276_MC1_BW_125			0x70
#define SX1276_MC1_BW_250			0x80
#define SX1276_MC1_BW_500			0x90
#define SX1276_MC1_CR_4_5			0x02
#define SX1276_MC1_CR_4_6			0x04
#define SX1276_MC1_CR_4_7			0x06
#define SX1276_MC1_CR_4_8			0x08

#define SX1276_MC1_IMPLICIT_HEADER_MODE_ON      0x01
// transmit power configuration for RegPaConfig
#define SX1276_PAC_PA_SELECT_PA_BOOST           0x80
#define SX1276_PAC_PA_SELECT_RFIO_PIN           0x00
#define SX1276_PAC_MAX_POWER_MASK               0x70

// the bits to change for max power.
#define SX127X_PADAC_POWER_MASK                 0x07
#define SX127X_PADAC_POWER_NORMAL               0x04
#define SX127X_PADAC_POWER_20dBm                0x07

// convert milliamperes to equivalent value for
// RegOcp; delivers conservative value.
#define SX127X_OCP_MAtoBITS(mA)                 \
    ((mA) < 45   ? 0 :                          \
     (mA) <= 120 ? ((mA) - 45) / 5 :            \
     (mA) < 130  ? 0xF :                        \
     (mA) < 240  ? ((mA) - 130) / 10 + 0x10 :   \
                   27)

// bit in RegOcp that enables overcurrent protect.
#define SX127X_OCP_ENA                          0x20
                                                    
// sx1276 RegModemConfig2          
#define SX1276_MC2_RX_PAYLOAD_CRCON		0x04

// sx1276 RegModemConfig3          
#define SX1276_MC3_LOW_DATA_RATE_OPTIMIZE	0x08
#define SX1276_MC3_AGCAUTO			0x04

// preamble for lora networks (nibbles swapped)
#define LORA_MAC_PREAMBLE			0x34

#define RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG1    0x0A
#define RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG2    0x70	// for SX1276 only

//-----------------------------------------
// Parameters for RSSI monitoring
#define SX127X_FREQ_LF_MAX      525000000       // per datasheet 6.3
#define RF_MID_BAND_THRESH	SX127X_FREQ_LF_MAX

// Constant values need to compute the RSSI value
// SX1276: per datasheet 5.5.3 and 5.5.5:
#define SX1276_RSSI_ADJUST_LF           -164    // add to rssi value to get dB (LF)
#define SX1276_RSSI_ADJUST_HF           -157    // add to rssi value to get dB (HF)
#define RSSI_OFFSET_LF			SX1276_RSSI_ADJUST_LF
#define RSSI_OFFSET_HF			SX1276_RSSI_ADJUST_HF

// per datasheet 2.5.2 (but note that we ought to ask Semtech to confirm, because
// datasheet is unclear).
#define SX127X_RX_POWER_UP      us2osticks(500) // delay this long to let the receiver power up.
#define RF_SNR_THRESH			2	// min. SNR value in dB for a valid recepture
// ---------------------------------------- 
// Constants for radio registers
#define OPMODE_LORA			0x80
#define OPMODE_MASK			0x07
#define OPMODE_LFMASK			0x08

#define OPMODE_SLEEP			0x00
#define OPMODE_STANDBY			0x01
#define OPMODE_FSTX			0x02
#define OPMODE_TX			0x03
#define OPMODE_FSRX			0x04
#define OPMODE_RX			0x05
#define OPMODE_RX_SINGLE		0x06 
#define OPMODE_CAD			0x07
// ----------------------------------------
// LoRa opmode bits
#define OPMODE_LORA_SX127x_AccessSharedReg              (1u << 6)
#define OPMODE_LORA_SX1276_LowFrequencyModeOn           (1u << 3)

// ----------------------------------------
// Bits masking the corresponding IRQs from the radio
#define IRQ_LORA_RXTOUT_MASK		0x80
#define IRQ_LORA_RXDONE_MASK		0x40
#define IRQ_LORA_CRCERR_MASK		0x20
#define IRQ_LORA_HEADER_MASK		0x10
#define IRQ_LORA_TXDONE_MASK		0x08
#define IRQ_LORA_CDDONE_MASK		0x04
#define IRQ_LORA_FHSSCH_MASK		0x02
#define IRQ_LORA_CDDETD_MASK		0x01

#define IRQ_FSK1_MODEREADY_MASK         0x80
#define IRQ_FSK1_RXREADY_MASK           0x40
#define IRQ_FSK1_TXREADY_MASK           0x20
#define IRQ_FSK1_PLLLOCK_MASK           0x10
#define IRQ_FSK1_RSSI_MASK              0x08
#define IRQ_FSK1_TIMEOUT_MASK           0x04
#define IRQ_FSK1_PREAMBLEDETECT_MASK    0x02
#define IRQ_FSK1_SYNCADDRESSMATCH_MASK  0x01
#define IRQ_FSK2_FIFOFULL_MASK          0x80
#define IRQ_FSK2_FIFOEMPTY_MASK         0x40
#define IRQ_FSK2_FIFOLEVEL_MASK         0x20
#define IRQ_FSK2_FIFOOVERRUN_MASK       0x10
#define IRQ_FSK2_PACKETSENT_MASK        0x08
#define IRQ_FSK2_PAYLOADREADY_MASK      0x04
#define IRQ_FSK2_CRCOK_MASK             0x02
#define IRQ_FSK2_LOWBAT_MASK            0x01

// ----------------------------------------
// DIO function mappings                D0D1D2D3
#define MAP_DIO0_LORA_RXDONE            0x00  // 00------
#define MAP_DIO0_LORA_TXDONE            0x40  // 01------
#define MAP_DIO1_LORA_RXTOUT            0x00  // --00----
#define MAP_DIO1_LORA_NOP               0x30  // --11----
#define MAP_DIO2_LORA_NOP               0x0C  // ----11--

#define MAP_DIO0_FSK_READY              0x00  // 00------ (packet sent / payload ready)
#define MAP_DIO1_FSK_NOP                0x30  // --11----
#define MAP_DIO2_FSK_TXNOP              0x04  // ----01--
#define MAP_DIO2_FSK_TIMEOUT            0x08  // ----10--


// FSK IMAGECAL defines
#define RF_IMAGECAL_AUTOIMAGECAL_MASK	0x7F
#define RF_IMAGECAL_AUTOIMAGECAL_ON	0x80
#define RF_IMAGECAL_AUTOIMAGECAL_OFF	0x00  // Default

#define RF_IMAGECAL_IMAGECAL_MASK	0xBF
#define RF_IMAGECAL_IMAGECAL_START	0x40

#define RF_IMAGECAL_IMAGECAL_RUNNING	0x20
#define RF_IMAGECAL_IMAGECAL_DONE	0x00  // Default


// RADIO STATE

#define LNA_RX_GAIN                     (0x20|0x03) // MaxLNAGain + NoBoost
#define LNA_MAX_GAIN                    (0x20|0x03) // MaxLNAGain + Boost(150%) -> Default for SX1276

#endif /* REGSLORA_H */

