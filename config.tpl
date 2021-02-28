; BeeHive Logger template (!) config file for initialization 
;===============================================================================
; This file must be customized first wherever you have other GPIO numbers or 
; you see 'xxx' before renaming it to config.ini for final usage as project 
; runtime parameter set.
;===============================================================================

; BeeIoT Gateway&Server config file for first runtime initialization 
; 28.02.2021
VERSION = 2.5	; Version of Config.INI File Format
VMAJOR	= 1		; Major Version of BIoTWAN Protocol
VMINOR	= 4		; Minor Version of BIoTWAN Protocol

; BeeIoT Server configuration runtime parameters
; This file can be modified during runtime of beeiotgw !
; All Values are parsed cyclic during runtime right after the wait loop.
[HWCONFIG]              ; wPi configuration settings

; Component enabler
HCLORA          = 1	; =1 LoRa Port sending enabled
HCLORAWAN       = 0	; =0 BIoTWAN protocol enabled, =1 LoRaWAN enable (not supp. yet)
HCWIFICLIENT    = 1	; =1 Client (!) onboard Wifi to be used 
HCGPS           = 0	; =1 GPS module enabled
HCLOCWEB        = 1	; =1 Activate local Webpage date preparation at BEEIOTWEB
HCREMWEB        = 1	; =1 Activate remote Webpage date preparation at EXFTPURL > EXFTPPATH

; LoRa Modem Chip GPIo section
LORANUMCHN	= 2		; # of supported LORA Modems (=1: single channel gateway)
LORADEFCHN	= 0		; LORA ModemID for JOIN REQUESTs (of x = 0..MAXGW)

; LoRa Ports > Dragino LoRa HAT GPIO of SX1276 + GPS chips
; Common SPI GPIO lanes 
;			 >wPi#:		  BCM:	J40:
LORAxMISO	= 13	; GPIO 9  	Pin 21	MISO
LORAxMOSI	= 12	; GPIO 10 	Pin 19  MOSI
LORAxSCK	= 14	; GPIO 11 	Pin 23	SCLK 

; LoRa Port0 >wPi#:       BCM:	J40:	=> Radio Modem 0
LORA0CS		= 6		; GPIO 25  	Pin 22	NSS0
LORA0RST	= 0		; GPIO 0	Pin 11	Reset0
LORA0DIO0	= 7		; GPIO 4	Pin  7	DIO0-0
LORA0DIO1	= 4		; GPIO 23	Pin 16	DIO1-0 n.c.
LORA0DIO2	= 5		; GPIO 24	Pin 18	DIO2-0 n.c.
LORA0DIO3	= -1	; GPIO x	Pin x	DIO3-0 n.c.
LORA0DIO4	= -1	; GPIO x	Pin x	DIO4-0 n.c.
LORA0DIO5	= -1	; GPIO x	Pin x	DIO5-0 n.c.
LORA0CHANNEL= 0		; Channel cfg. set (see BeeIoTWAN.h) 

; LoRa Port1 >wPi#:	      BCM:	J40:	=> Radio Modem 1
LORA1CS		= 21	; GPIO 5  	Pin 29	NSS1
LORA1RST	= 22	; GPIO 6	Pin 31	Reset1
LORA1DIO0	= 23	; GPIO 13	Pin 33	DIO0-1
LORA1DIO1	= -1	; GPIO x	Pin x	DIO1-1 n.c.
LORA1DIO2	= -1	; GPIO x	Pin x	DIO2-1 n.c.
LORA1DIO3	= -1	; GPIO x	Pin x	DIO3-1 n.c.
LORA1DIO4	= -1	; GPIO x	Pin x	DIO4-1 n.c.
LORA1DIO5	= -1	; GPIO x	Pin x	DIO5-1 n.c.
LORA1CHANNEL= 1		; Channel cfg. set (see BeeIoTWAN.h)

GPS0TXD		= 15	; GPIO 14 	Pin  8 TxD	of GPS module
GPS0RXD		= 16	; GPIO 15 	Pin 10 RxD  of GPS module

; LoRa Client Registration:
; Node 1: BeeIoT ESP32-WROOM32:	MAC: 24:6F:28:D5:8A:DC	default: BeeHive Weightcell #1
ND1_GWID	= 1		; GW ID relative to GWIDx (1..LORANUMCHN)
ND1_MID		= 0		; Assign physical Radio Modem to virt. GW
ND1_APPEUI  = 1		; 1= BIOT, 2= TURTLE, 3= GH
ND1_DEVEUI1 = DC8AD5FF	; Unique DEVEUI of Node Upper Long
ND1_DEVEUI2 = FE286F24	; Unique DEVEUI of Node Lower Long
ND1_FREPORT = 10	; Report Frequency in Minutes
ND1_WCALIB	= 0		; Weight Cell calibration (absolute +/-)

;---------------------------------------------------
; Node 2: ESP32-WROVERB: MAC 24:6F:28:F0:0D:AC		beacon test Module 1
ND2_GWID	= 1		; GW ID relative to GWIDx (1..LORANUMCHN)
ND2_MID		= 0		; Assign physical Radio Modem to virt. GW
ND2_APPEUI  = 1		; 1= BIOT, 2= TURTLE, 3= GH
ND2_DEVEUI1 = AC0DF0FF	; Unique DEVEUI of Node Upper Long
ND2_DEVEUI2 = FE286F24	; Unique DEVEUI of Node Lower Long
ND2_FREPORT = 1	    ; Report Frequency in Minutes
ND2_WCALIB	= 0		; Weight Cell calibration (absolute +/-)

;---------------------------------------------------
; Node 3: BeeIoT ESP32-WROOM32:	MAC: 94:FE:8A:B5:AA:8C	Beacon test Module 2
ND3_GWID	= 1		; GW ID relative to GWIDx (1..LORANUMCHN)
ND3_MID		= 0		; Assign physical Radio Modem to virt. GW
ND3_APPEUI  = 1		; 1= BIOT, 2= TURTLE, 3= GH
ND3_DEVEUI1 = 94FE8AFF	; Unique DEVEUI of Node Upper Long
ND3_DEVEUI2 = FEB5AA8C	; Unique DEVEUI of Node Lower Long
ND3_FREPORT = 1	    ; Report Frequency in Minutes
ND3_WCALIB	= 0		; Weight Cell calibration (absolute +/-)

;---------------------------------------------------
; Node 4: BeeIoT ESP32-WROOM32:	MAC: 2C:2B:16:28:6F:24 	Beehive Weight cell test Module 3
ND4_GWID	= 2		; GW ID relative to GWIDx (1..LORANUMCHN)
ND4_MID		= 1		; Assign physical Radio Modem to virt. GW
ND4_APPEUI  = 1		; 1= BIOT, 2= TURTLE, 3= GH
ND4_DEVEUI1 = 2C2B16FF	; Unique DEVEUI of Node Upper Long
ND4_DEVEUI2 = FE286F24	; Unique DEVEUI of Node Lower Long
ND4_FREPORT = 10	; Report Frequency in Minutes
ND4_WCALIB	= 0		; Weight Cell calibration (absolute +/-)


[BEEIOT]   ; Init of Main Programm
BHLOOPWAIT      = 10          ; Sensor loop wait time in Minutes
BEEIOTHOME      = /home/pi/biot  ; Home path for beeiot housekeeping data
LOGFILE         = beeiot.txt ; log file name (/w extension)
CSVFILE         = beeiotlog  ; *.csv data log file name (/wo extension): results in beeiotlogYYYY.csv
CSVDAYS         = beeiotdays ; *.csv file of daily statistic summary (/wo extension)
ACTIOFAIL       = 1          ; allowed action on IO Error: 0= no Action, 1= exit, 2=reboot
VERBOSE         = 1          ; verbose levels +1=main flow + 2=OneWire + 4=hx711 + 8 ePaper
                             ; 16=LANcfg + 32=SDCard + 64=ADS + 128=SPI Port
                             ; 256=Lora-Radio + 512=Lora-BIoTWAN-Protocol

[WIFI]      ; Client side Wifi settings
WIFISSID        = 'sssss'			; Client/Server Wifi SSID (32 By.)
WIFIKEY         = 'ppppppppp'		; Client/Server Wifi KEY (32 By.)

[WEBUI]
AUTOUPDATE      = 1                 ; init automatic update of FTP-& web site (=0 disabled)
BEELOGWEB       = /var/www/beeiot   ; root path to webserver home of beelog for log & data files
BEEKEEPER       = 'xxx'             ; Full name of Owner/User/BeeKeeper (not used yet)
LOCDAT1         = '-Garten-'        ; Location of BeeHive1 (not used yet)
LOCDAT2         = 'xxx'             ; Street (not used yet)
LOCPLZ          = xxx               ; ZIP code of location (also fro weather data from web)   (not used yet)
LOCDAT3         = 'xxx'             ; location name (not used yet)
PICSMALL        = xxx.jpg           ; Pic of BeeHive (compressed) for WebLogo
PICLARGE        = xxx.jpg           ; Pix of Beehive full size
WEBDEFFILE      = index.html        ; default Web index file to be updated
NOTICEFILE      = beenote.txt       ; text file of service notices for logging
ALARMON         = 0                 ; =1 Global "ALARM enabled" for security/events
ALARMWEIGHT     = 0                 ; Alarm on Weight change > 50% in 5 seconds: thieve
                                    ; =0 disabled, 1..99% enabled, typical 50(%)
ALARMSWARM      = 0                 ; Alarm on weight change > 10% in 10 minutes: swarm
                                    ; =0 disabled, 1..99% enabled, typical 10(%)
ALARMBATT1      = 0                 ; =0 disabled; 1..99% enabled, typical 100%= 4.2V ->Max
ALARMBATT2      = 0                 ; =0 disabled; 1..99% enabled, typical 0%  = 3.3V ->Off

[EXPORT]
EXFTPURL        = ftp.xxx.de        ; FTP site URL for upload of raw logger data from BEELOGWEB
EXFTPPORT       = 21                ; Portnumber of URL (used as string)
EXFTPPATH       = xxx/beelog        ; relative FTP path to URL
EXFTPPROXY      =                   ; If needed: FTP proxy server; set '' if no proxy needed
EXFTPPROXYPORT  =                   ; used proxy port (used as string)
EXFTPUSER       =                   ; no user name for FTP access (get pwd by dialogue or local .netrc file)
BKUPPATH        = /xxx/xxx          ; Backup file path (local or remote) (not used yet)
BKUPFILE        = beelog.bak        ; name of config/log backup file at BKUPPATH (not used yet)
