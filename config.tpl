; BeeHive Logger template (!) config file for initialization 
;===============================================================================
; This file must be customized first wherever you have other GPIO numbers or 
; you see 'xxx' before renaming it to config.ini for final usage as project 
; runtime parameter set.
;===============================================================================

; BeeIoT Gateway&Server config file for first runtime initialization 
; 24.04.2023
VERSION = 2.8   ; Version of Config.INI File Format
VMAJOR	= 1     ; Major Version of BIoTWAN Protocol
VMINOR	= 9     ; Minor Version of BIoTWAN Protocol

; BeeIoT Server configuration runtime parameters
; This file can be modified during runtime of beeiotgw !
; All Values are parsed cyclic during runtime right after the wait loop.
[HWCONFIG]              ; wPi configuration settings

; Component enabler
HCLORA          = 1	; =1 GW LoRa Port sending enabled
HCLORAWAN       = 0	; =0 BIoTWAN protocol enabled, =1 LoRaWAN enable (not supp. yet)
HCWIFICLIENT    = 0	; =1 GW onboard WiFI Client (!) enabled 
HCGPS           = 0	; =1 GW GPS module enabled
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
;---------------------------------------------------
; BIoT Beehive Weight cell test Boards PCB:v4.0
; ---------------------------------------------
; PCB#	     MAC:
; #01	6E136D8DF9D4
; #02	CA816D8DF9D4
; #03	78136D8DF9D4
; #04	E2816D8DF9D4
; #05	86136D8DF9D4
; #06	7E136D8DF9D4
; #07	EC816D8DF9D4
; #08	9C136D8DF9D4
; #09	90136D8DF9D4
; #10	62136D8DF9D4

; Prototype test boards
; #11   246F28D58ADC	ESP32 WROOM "Neue Wiese"

;---------------------------------------------------
; Node 1: BeeIoT ESP32S2-Mini:	Beehive Weight cell test Board 1
ND1_GWID	    = 2			; GW ID relative to GWIDx (1..LORANUMCHN)
ND1_MID         = 1			; Assign physical Radio Modem to virt. GW
ND1_APPEUI  	= 1			; 1= BIOT, 2= TURTLE, 3= GH
ND1_DEVEUI1 	= 6E136DFF	; Unique DEVEUI of Node Upper Long
ND1_DEVEUI2 	= FE8DF9D4	; Unique DEVEUI of Node Lower Long
ND1_FREPORT 	= 10		; Report Frequency in Minutes
ND1_WCALIB	    = -7748 	; Weight Cell calibration (absolute +/-) in Gramm
ND1_HWCONF  	= 7     	; EPD=+1, LORA=+2, SD=+4, WIFI=+8, NTP=+16, Beacon=+32
;--------------------------------------------------
; Node 2: BeeIoT ESP32S2-Mini:	Beehive Weight cell test Board 2
ND2_GWID		= 2			; GW ID relative to GWIDx (1..LORANUMCHN)
ND2_MID			= 1			; Assign physical Radio Modem to virt. GW
ND2_APPEUI  	= 1			; 1= BIOT, 2= TURTLE, 3= GH
ND2_DEVEUI1 	= CA816DFF	; Unique DEVEUI of Node Upper Long
ND2_DEVEUI2 	= FE8DF9D4	; Unique DEVEUI of Node Lower Long
ND2_FREPORT 	= 10	   	; Report Frequency in Minutes
ND2_WCALIB		= -7748		; Weight Cell calibration (absolute +/-) in Gramm
ND2_HWCONF  	= 7     	; EPD=+1, LORA=+2, SD=+4, WIFI=+8, NTP=+16, Beacon=+32
;---------------------------------------------------
; Node 3: BeeIoT ESP32S2-Mini:	Beehive Weight cell test Board 3
ND3_GWID	    = 2			; GW ID relative to GWIDx (1..LORANUMCHN)
ND3_MID		    = 1			; Assign physical Radio Modem to virt. GW
ND3_APPEUI  	= 1			; 1= BIOT, 2= TURTLE, 3= GH
ND3_DEVEUI1 	= 78136DFF	; Unique DEVEUI of Node Upper Long
ND3_DEVEUI2 	= FE8DF9D4	; Unique DEVEUI of Node Lower Long
ND3_FREPORT 	= 10	    ; Report Frequency in Minutes
ND3_WCALIB	    = -7748		; Weight Cell calibration (absolute +/-) in Gramm
ND3_HWCONF  	= 7     	; EPD=+1, LORA=+2, SD=+4, WIFI=+8, NTP=+16, Beacon=+32
;---------------------------------------------------
; Node 4: BeeIoT ESP32S2-Mini:	Beehive Weight cell test Board 4
ND4_GWID	    = 2			; GW ID relative to GWIDx (1..LORANUMCHN)
ND4_MID		    = 1			; Assign physical Radio Modem to virt. GW
ND4_APPEUI  	= 1			; 1= BIOT, 2= TURTLE, 3= GH
ND4_DEVEUI1 	= E2816DFF	; Unique DEVEUI of Node Upper Long
ND4_DEVEUI2 	= FE8DF9D4	; Unique DEVEUI of Node Lower Long
ND4_FREPORT 	= 10		; Report Frequency in Minutes
ND4_WCALIB	    = -7748		; Weight Cell calibration (absolute +/-) in Gramm
ND4_HWCONF  	= 7     	; EPD=+1, LORA=+2, SD=+4, WIFI=+8, NTP=+16, Beacon=+32
;---------------------------------------------------
; Node 5: BeeIoT ESP32S2-Mini:	Beehive Weight cell test Board 5
ND5_GWID	    = 2			; GW ID relative to GWIDx (1..LORANUMCHN)
ND5_MID		    = 1			; Assign physical Radio Modem to virt. GW
ND5_APPEUI  	= 1			; 1= BIOT, 2= TURTLE, 3= GH
ND5_DEVEUI1 	= 86136DFF 	; Unique DEVEUI of Node Upper Long Mini-1 Board#2
ND5_DEVEUI2 	= FE8DF9D4 	; Unique DEVEUI of Node Lower Long
ND5_FREPORT 	= 10 		; Report Frequency in Minutes
ND5_WCALIB	    = -7748		; Weight Cell calibration (absolute +/-) in Gramm
ND5_HWCONF  	= 7     	; EPD=+1, LORA=+2, SD=+4, WIFI=+8, NTP=+16, Beacon=+32
;---------------------------------------------------
; Node 6: BeeIoT ESP32S2-Mini:	Beehive Weight cell test Board 6
ND6_GWID		= 2			; GW ID relative to GWIDx (1..LORANUMCHN)
ND6_MID			= 1			; Assign physical Radio Modem to virt. GW
ND6_APPEUI  	= 1			; 1= BIOT, 2= TURTLE, 3= GH
ND6_DEVEUI1 	= 7E136DFF	; Unique DEVEUI of Node Upper Long
ND6_DEVEUI2 	= FE8DF9D4	; Unique DEVEUI of Node Lower Long
ND6_FREPORT 	= 10	    ; Report Frequency in Minutes
ND6_WCALIB		= -7748		; Weight Cell calibration (absolute +/-) in Gramm
ND6_HWCONF  	= 7     	; EPD=+1, LORA=+2, SD=+4, WIFI=+8, NTP=+16, Beacon=+32
;---------------------------------------------------
; Node 7: BeeIoT ESP32S2-Mini:	Beehive Weight cell test Board 7
ND7_GWID		= 2			; GW ID relative to GWIDx (1..LORANUMCHN)
ND7_MID			= 1			; Assign physical Radio Modem to virt. GW
ND7_APPEUI  	= 1			; 1= BIOT, 2= TURTLE, 3= GH
ND7_DEVEUI1 	= EC816DFF	; Unique DEVEUI of Node Upper Long
ND7_DEVEUI2 	= FE8DF9D4 	; Unique DEVEUI of Node Lower Long
ND7_FREPORT 	= 10 		; Report Frequency in Minutes
ND7_WCALIB		= -7748		; Weight Cell calibration (absolute +/-) in Gramm
ND7_HWCONF  	= 7     	; EPD=+1, LORA=+2, SD=+4, WIFI=+8, NTP=+16, Beacon=+32
;--------------------------------------------------
; Node 8: BeeIoT ESP32S2-Mini:	Beehive Weight cell test Board 8
ND8_GWID	    = 2			; GW ID relative to GWIDx (1..LORANUMCHN)
ND8_MID		    = 1			; Assign physical Radio Modem to virt. GW
ND8_APPEUI  	= 1			; 1= BIOT, 2= TURTLE, 3= GH
ND8_DEVEUI1 	= 9C136DFF	; Unique DEVEUI of Node Upper Long
ND8_DEVEUI2 	= FE8DF9D4	; Unique DEVEUI of Node Lower Long
ND8_FREPORT 	= 10	   	; Report Frequency in Minutes
ND8_WCALIB	    = -7748		; Weight Cell calibration (absolute +/-) in Gramm
ND8_HWCONF  	= 7     	; EPD=+1, LORA=+2, SD=+4, WIFI=+8, NTP=+16, Beacon=+32
;--------------------------------------------------
; Node 9: BeeIoT ESP32S2-Mini:	Beehive Weight cell test Board 9
ND9_GWID	    = 2			; GW ID relative to GWIDx (1..LORANUMCHN)
ND9_MID	        = 1			; Assign physical Radio Modem to virt. GW
ND9_APPEUI  	= 1			; 1= BIOT, 2= TURTLE, 3= GH
ND9_DEVEUI1 	= 90136DFF	; Unique DEVEUI of Node Upper Long
ND9_DEVEUI2 	= FE8DF9D4	; Unique DEVEUI of Node Lower Long
ND9_FREPORT 	= 10	   	; Report Frequency in Minutes
ND9_WCALIB	    = -7748		; Weight Cell calibration (absolute +/-) in Gramm
ND9_HWCONF  	= 7     	; EPD=+1, LORA=+2, SD=+4, WIFI=+8, NTP=+16, Beacon=+32
;--------------------------------------------------
; Node 10: BeeIoT ESP32S2-Mini:	Beehive Weight cell test Board 3
ND10_GWID	    = 2			; GW ID relative to GWIDx (1..LORANUMCHN)
ND10_MID        = 1			; Assign physical Radio Modem to virt. GW
ND10_APPEUI  	= 1			; 1= BIOT, 2= TURTLE, 3= GH
ND10_DEVEUI1 	= 62136DFF	; Unique DEVEUI of Node Upper Long
ND10_DEVEUI2 	= FE8DF9D4	; Unique DEVEUI of Node Lower Long
ND10_FREPORT 	= 10	   	; Report Frequency in Minutes
ND10_WCALIB	    = -7748		; Weight Cell calibration (absolute +/-) in Gramm
ND10_HWCONF  	= 7     	; EPD=+1, LORA=+2, SD=+4, WIFI=+8, NTP=+16, Beacon=+32

;--------------------------------------------------
; Node 11: BeeIoT ESP32-WROOM32:	MAC: 24:6F:28:D5:8A:DC	default: BeeHive Weightcell #1
ND11_GWID	    = 1			; GW ID relative to GWIDx (1..LORANUMCHN)
ND11_MID		= 0			; Assign physical Radio Modem to virt. GW
ND11_APPEUI  	= 1			; 1= BIOT, 2= TURTLE, 3= GH
ND11_DEVEUI1 	= DC8AD5FF	; Unique DEVEUI of Node Upper Long
ND11_DEVEUI2 	= FE286F24	; Unique DEVEUI of Node Lower Long
ND11_FREPORT 	= 10		; Report Frequency in Minutes
ND11_WCALIB	    = -12700 	; Weight Cell calibration (absolute +/-) in Gramm
ND11_HWCONF  	= 3     	; EPD=+1, LORA=+2, SD=+4, WIFI=+8, NTP=+16, Beacon=+32
;--------------------------------------------------;---------------------------------------------------

[BEEIOT]   ; Init of Main Programm
BHLOOPWAIT      = 10          ; Sensor loop wait time in Minutes
BEEIOTHOME      = /home/pi/biot  ; Home path for beeiot housekeeping data
LOGFILE         = beeiot.txt ; log file name (/w extension)
CSVFILE         = beeiotlog  ; *.csv data log file name (/wo extension): results in beeiotlogYYYY.csv
CSVDAYS         = beeiotdays ; *.csv file of daily statistic summary (/wo extension)
ACTIOFAIL       = 1          ; allowed action on IO Error: 0= no Action, 1= exit, 2=reboot
VERBOSE         = 1          ; verbose levels +1=main flow + 2=OneWire + 4=hx711 + 8 ePaper
                             ; 16=LANcfg + 32=SDCard + 64=ADS + 128=SPI Port
                             ; 256=Lora-Radio + 512=Lora-BIoTWAN-Protocol +
							 ; 1024=MsgQueue & MsgBuffer class handling
CMDFILE         = beecmd.txt        ; Command file for RX1 response window
RESULTFILE      = beeresult.txt     ; Result data from RX1-command

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
