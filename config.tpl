; BeeHive Logger template (!) config file for initialization 
;===============================================================================
; This file must be customized first wherever you have other GPIO numbers or 
; you see 'xxx' before renaming it to config.ini for final usage as project 
; runtime parameter set.
;===============================================================================

; BeeIoT Gateway&Server config file for first runtime initialization 
; 01.02.2020
VERSION = 2.3		; Version of Config.INI File Format
VMAJOR	= 1		; Major Version of BIoTWAN Protocol
VMINOR	= 2		; Minor Version of BIoTWAN Protocol

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
LORANUMCHN	= 1		; # of supported LORA Modems =1: single channel gateway
LORADEFCHN	= 0		; LORA ModemID for JOIN REQUESTs (of x = 0..MAXGW)

; LoRa Ports > Dragino LoRa HAT GPIO of SX1276 + GPS chips
; Common SPI GPIO lanes 
;			 >wPi#:		  BCM:	J40:
LORAxMISO	= 13	; GPIO 9  	Pin 21	MISO
LORAxMOSI	= 12	; GPIO 10 	Pin 19  MOSI
LORAxSCK	= 14	; GPIO 11 	Pin 23	SCLK 

; LoRa Port0 >wPi#:       BCM:	J40:
LORA0CS		= 6		; GPIO 25  	Pin 22	NSS0
LORA0RST	= 0		; GPIO 0	Pin 11	Reset0
LORA0DIO0	= 7		; GPIO 4	Pin  7	DIO0-0
LORA0DIO1	= 4		; GPIO 23	Pin 16	DIO1-0 n.c.
LORA0DIO2	= 5		; GPIO 24	Pin 18	DIO2-0 n.c.
LORA0DIO3	= -1	; GPIO x	Pin x	DIO3-0 n.c.
LORA0DIO4	= -1	; GPIO x	Pin x	DIO4-0 n.c.
LORA0DIO5	= -1	; GPIO x	Pin x	DIO5-0 n.c.
LORA0CHANNEL= 0		; Channel cfg. set (see BeeIoTWAN.h) 

; LoRa Port1 >wPi#:	      BCM:	J40:
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

; Component enabler
HCLORA          = 1     ; =1 LoRa Port sending enabled
HCLORAWAN       = 0	; =0 BIoTWAN protocol enabled, =1 LoRaWAN enable (not supp. yet)
HCGPS           = 0	; =1 GPS module enabled
HCLOCWEB        = 1	; =1 Activate local Webpage date preparation at BEEIOTWEB
HCREMWEB        = 1	; =1 Activate remote Webpage date preparation at EXFTPURL > EXFTPPATH


[BEEIOT]   ; Init of Main Programm
BHLOOPWAIT      = 1          ; Sensor loop wait time in Minutes
BEEIOTHOME      = /home/pi/biot  ; Home path for beeiot housekeeping data
LOGFILE         = beeiot.txt ; log file name (/w extension)
CSVFILE         = beeiotlog  ; *.csv data log file name (/wo extension): results in beeiotlogYYYY.csv
CSVDAYS         = beeiotdays ; *.csv file of daily statistic summary (/wo extension)
ACTIOFAIL       = 1          ; allowed action on IO Error: 0= no Action, 1= exit, 2=reboot
VERBOSE         = 1          ; verbose levels +1=main flow + 2=OneWire + 4=hx711 + 8 ePaper
                             ; 16=LANcfg + 32=SDCard + 64=ADS + 128=SPI Port
                             ; 256=Lora-Radio + 512=Lora-BIoTWAN-Protocol

[HX711]    ; Init of Weight scale ADC
TARA            = 297570     ; Calibration for 0 kg (offset of cover weight = ~6,75kg)
TARASET         = 0          ; =1 TARA reset by last measured weight level -> at next loop
REFKG           = 44000      ; weight scale reference value of 1kg of Bosche H40A with 2mV/V sensibility
TEMPCOMP        = 1.0        ; Temp. compensation factor per Grad           
NSAMPLES        = 1          ; Number of read loops for average calculation (Max Range 2..100)

[ONEWIRE]  ; Init of One-Wire Temp.-devices
TEMPCEXT        = 1.00       ; temperature compensation External sensor
TEMPCINT        = 1.00       ; temperature compensation Scale-H40A sensor
TEMPCHIVE       = 1.00       ; temperature compensation InternalHive sensor

[WIFI]      ; Client side Wifi settings
WIFISSID        = 'sssss'			; Client/Server Wifi SSID (32 By.)
WIFIKEY         = 'ppppppppp'		; Client/Server Wifi KEY (32 By.)

[WEBUI]
AUTOUPDATE      = 1                 ; init automatic update of FTP-& web site (=0 disabled)
BEELOGWEB       = /var/www/beelog   ; root path to webserver home of beelog for log & data files
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
