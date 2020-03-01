; BeeHive Logger template (!) config file for initialization 
;===============================================================================
; This file must be customized first wherever you have other GPIO numbers or 
; you see 'xxx' before renaming it to config.ini for final usage as project 
; runtime parameter set.
;===============================================================================

; 03.01.2019
VERSION	= 2.2
; Beehive tool configuration runtime parameter set
;
; This file is target to be modified during runtime by beehive program !
; All Values are parsed cyclic during runtime right after the wait loop.

[HWCONFIG]                 ; wPi-GPIO configuration settings (BCM GPIO#)
PINHX711DT = 23            ; J8.P16: HX711 DT  dataline
PINHX711CLK= 20            ; J8.P38: HX711 SCK clockline (SPI1 MOSI)
PINADSALERT= 22            ; J8.P15: ADS ALERT/RDY GPIO22
PINTESTLED = 12            ; J8.P32: TEST LED (red) /w ext. pullup
PINTARABT  = 16            ; J8.P36; TARA Reset ISR trigger Button
PINOWIOPIN = 4             ; J8.P07: OneWire dataline -> claimed by kernel cfg files => do not change !

	; Display: WaveShare 2.7" ePaper HAT (DISPLAY =1) SPI0 => wPi setup with GPIO#
EPAPERDIN  = 10            ; J8.P19 SPI0-MOSI GPIO10
EPAPERCLK  = 11            ; J8.P23 SPI0-SCLK GPIO11
EPAPERCS   = 8             ; J8.P24 SPI0-CE0  GPIO08
EPAPERDC   = 25            ; J8.P22 GPIO25
EPAPERRST  = 17            ; J8.P11 GPIO17
EPAPERBUSY = 24            ; J8.P18 GPIO24
EPAPERKEY1 = 5             ; J8.P29 GPIO05
EPAPERKEY2 = 6             ; J8.P31 GPIO06
EPAPERKEY3 = 13            ; J8.P33 GPIO13
EPAPERKEY4 = 26            ; J8.P37 GPIO26

	; Component enabler
GPSMOUSE   = 0             ; =1 GPS mouse connected -> part of logdata (not used yet)
THERMOCAM  = 0             ; =1 Thermo camera connected >0: save pics each x min. (not used yet)
THERMOLOOP = 10            ; Wait time for thermo pic creation in minutes (not used yet)
BEECOUNTER = 0             ; =1 BeeCounter at hive entry connected -> part of logdata  (not used yet)
DISPLAY    = 1             ; =1 Activate local display update => =1 for WaveShare ePaper 2.7"
WEBUI      = 1             ; =1 Activate Webpage date preparation at BEELOGWEB

[BEELOG]   ; Init of Main Programm
BHLOOPWAIT = 600           ; loop wait time in Sec. (600 = 10Min.)
BEELOGHOME = /xxx/xxx	   ; Home path for beelog housekeeping data
LOGFILE	   = beelog.txt    ; log file name (/w extension)
CSVFILE    = beelog        ; *.csv data log file name (/wo extension)
CSVDAYS    = beedays	   ; *.csv file of daily statistic summary (/wo extension)
ACTIOFAIL  = 1             ; allowed action on IO Error: 0= no Action, 1= exit, 2=reboot
VERBOSE	   = 65            ; verbose levels +1:main flow + 2=1-wire + 4+8=hx711(lev 1+2)
                           ; 16=Import/export + 32=Web Update + 64=raspimon
                           ; +128 = statistic data calculation

[HX711]    ; Init of Weight scale ADC
TARA       = xxx	       ; Calibration for 0 kg (e.g. 823500)
TARARESLOCK= 0             ; =0 TARA reset button is disabled, =1 enabled  
TARARESET  = 0             ; =0 TARA Button active; =1 TARA Button simulated
REFKG      = xxx           ; weight scale reference value of 1kg (e.g. 46350)
HXSPREAD   = 10            ; trust range of weight scale +/- 5% = 10 per measurement
DATSPREAD  = 10            ; trust range of weight scale +/- 5% = 10 between 2 measurements
TEMPCOMP   = xxx           ; Temp. compensation factor per Grad (e.g. 0.9)
NSAMPLES   = 10            ; Number of read loops for average calculation 
                           ; Max Range: 2..100

[ONEWIRE]  ; Init of One-Wire devices
OWFILEPATH   = /sys/bus/w1/devices ; path to one wire filesystem of kernel driver
OWTEMPEXTID  = xxx				; OW ID of external DS18B20 sensor 1
TEMPCEXT     = 1.00             ; temperature compensation External sensor
OWTEMPINTID  = xxx				; OW ID of internal DS18B20 sensor 2 -> used as beelog UID
TEMPCINT     = 0.00             ; temperature compensation Internal sensor
OWTEMPHIVEID1= xxx				; OW ID of  WScale1 DS18B20 sensor 3
TEMPCHIVE1   = 0.00             ; temperature compensation Hive1 sensor
OWTEMPHIVEID2= xxx				; OW ID of  WScale2 DS18B20 sensor 4
TEMPCHIVE2   = 0.00             ; temperature compensation Hive2 sensor

[WEBUI]
AUTOUPDATE  = 1                 ; init automatic update of FTP-& web site (=0 disabled)
BEELOGWEB   = /var/www/beelog   ; root path to webserver home of beelog for log & data files
BEEKEEPER   = 'xxx'				; Full name of Owner/User/BeeKeeper (not used yet)
LOCDAT1     = '-Garten-'        ; Location of BeeHive1 (not used yet)
LOCDAT2     = 'xxx'             ; Street (not used yet)
LOCPLZ      = xxx               ; ZIP code of location (also fro weather data from web)   (not used yet)
LOCDAT3     = 'xxx'             ; location name (not used yet)
PICSMALL    = xxx.jpg           ; Pic of BeeHive (compressed) for WebLogo
PICLARGE    = xxx.jpg           ; Pix of Beehive full size
WEBDEFFILE  = index.html        ; default Web index file to be updated
NOTICEFILE  = beenote.txt       ; text file of service notices for logging
ALARMON     = 0                 ; =1 Global "ALARM enabled" for security/events (not used yet)
ALARMWEIGHT = 0                 ; Alarm on Weight change > 50% in 5 seconds: thieve (not used yet)
                                ; =0 disabled, 1..99% enabled, typical 50(%)
ALARMSWARM  = 0                 ; Alarm on weight change > 10% in 10 minutes: swarm (not used yet)
                                ; =0 disabled, 1..99% enabled, typical 10(%)
ALARMBATT1  = 0                 ; =0 disabled; 1..99% enabled, typical 98(%)= 4.9V/12,8V (not used yet)
ALARMBATT2  = 0                 ; =0 disabled; 1..99% enabled, typical 95(%)= 4.75V/12,5V (not used yet)

[EXPORT]
EXFTPURL    = ftp.xxx.de        ; FTP site URL for upload of raw logger data from BEELOGWEB
EXFTPPORT   = 21                ; Portnumber of URL (used as string)
EXFTPPATH   = xxx/beelog        ; relative FTP path to URL
EXFTPPROXY  =                   ; If needed: FTP proxy server; set '' if no proxy needed
EXFTPPROXYPORT =                ; used proxy port (used as string)
EXFTPUSER   =                   ; no user name for FTP access (get pwd by dialogue or local .netrc file)
BKUPPATH    = /xxx/xxx          ; Backup file path (local or remote) (not used yet)
BKUPFILE    = beelog.bak        ; name of config/log backup file at BKUPPATH (not used yet)
