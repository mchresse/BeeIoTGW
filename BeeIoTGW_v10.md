# BeeIoT-Gateway v1.0
### Der Gateway zur BeeIoT Bienenstockwaage (auf RPi)
<img src="./images_v2/BeeLogFront.jpg" width=250>

**01.01.2020 by Randolph Esser**

---
## Inhaltsverzeichnis:
- [Einführung](#einführung)
	* [Die Programm Struktur](#die-programm-struktur)
		+ [Setup Phase](setup-phase)
		+ [Loop Phase](loop-phase)
	* [Optional: WebUI Daten Service](#optional:-webui-daten-service)
- [WebUI ToDo - Liste](#webui-todo---liste)
- [Ideen Liste](#ideen-liste)
<!-- toc -->
---


## Einführung
Nach der Beschreinung der BeeIot Sensor Knoten auf Basis des ESP32 chipset, nun di passende Gegenstelle auf Raspberry Pi Basis mit den Rollen des (ähnlich wie bei LoRaWan):
- Gateway Server
	- Entgegennahme der BIoT LoRa Pakete (Uplink) auf der vereinbarten Modem Channel configuration
	- Versenden von BIoT Antwort Paketen.
- NetworkServer 
	- Koordination der RX/TX Paket 
	- Bedienung des RX1 Commando Fensters im Anschluss an jedes Node Paket (optional)
- JOIN Server
	- Evaluierung von JOIN request auf Basis einer statischen DevUID Referenztabelle
	- Verwaltung einer dynamischen Node Tabelle incl RX/Queue.
- Application Server
	- Auswertung des Frame-Payloads applikationsspezifisch nach zugewiesener AppEUI.
	- Default App: BIoT zur Auswertung der BeeIoT Node Sensordaten 
	- Weiterleitung der validierten Datenpakete an die Webdarstellung in Form einer CSV Datei


#### LoRa WAN Support - Node side
Für die remote Connection ohne "stromfressenden" WiFi Betrieb oder nicht-erreichbarem Hotspot, ist ein LoRa Funktmodul vorgesehen. Auf 868MHz voreingestellt kann es abhängig von der räumlichen Topologie Reichweiten bis zu 8km ermöglichen.

Das LoRa-WAN Protokoll ist auf geringe Band-Belastung und geringem Stromverbrauch ausgelegt.

Das verbaute Funkmodul bietet erst einmal nur den LoRa-MAC layer Übertragungssupport. Den Rest für das **[LoRaWAN Protokoll](https://lora-alliance.org/about-lorawan)** leisten die dazugehörigen Bibliotheken (z.B. die OSS Lib: LMIC von IBM -> search in GitHub).
Darin sind dann eine Peer2Peer Verbindung über unique Sender/Empfänger IDs und Verschlüsselungs-keys ähnlich wie bei TCP/IP kombiniert mit SSH enthalten.
Es zeigten sich bei der Migration der LMIC Lib Instablitäten des LoRa Modes -> es wurden immer wieder FSK Mode IRQs empfangen, was die LoRa Statusführung durcheinander bringt.
Auch ist das von IBM gewählte OS layer Model nicht so handsam wie erwartet.
In Summe stellte die LMIC-Lib für den ESP32 in Kombination mit den übrigen Aktionen zur Sensorbehandlung zumindest auf Node/Cient Seite einen ziemlichen overhead dar, der aber leider nötig ist um das vollständige LoRa-WAN Protokoll nach Spezifikation zu erfüllen (Band-Hopping, Encryption, OTAA joining usw.).
Als Gegenstück ist ein RaspberryPi basierter Gateway vorgesehen, der seinerseits wieder die benötigte WiFi/LAN Anbindung hat, um die gewonnenen Daten zu validieren und auf eine Website zu spiegeln.
Da ein GW ggfs. mit mehreren Clients quasi-gleichzeitig zu tun hat, ist die LMIC STack implementierung optimal. Der OS Layer nimmt einem hier das Queueing hereinkommender Pakete sowie die protokollgerechte Quittierung und Bandmanagement vollständig ab.
Ein weiterer interessanter Client sample Code findet sich über eine Beispielprojekt des Opennet teams:
https://wiki.opennet-initiative.de/wiki/LoRaSensor .

Aktuell befinden sich der Code dazu mangels vollwertigem Gateway noch im Beta-Stadium (!):
Soll heisen auf basis des RADIO layers von LMIC habe ich ein eigenes "schmalspur" WAN Protokoll entworfen (BeeIoT-WAN) welches die Paket Kommunikation auf den rudimentären Austausch von Sensordaten mit einfacher Quittierung (zunächst ohne Multi-Bandmanagement oder Encryption) "optimiert".
Auf dieser Basis habe ich die ESP32 Client- und RaspberryPi GW-seitigen Module entworfen.
Kerneigenschaften der Module:
- Erkennung durch Sender/Empfänger-IDs im "BeeIoT-WAN"-eigenen Header (nur 5 Bytes)
- CRC basierte Datenprüfung
- automatische Quittierung und resend/retry Kommunikation als Flow Control

Was noch fehlt:
- Collison detection (aber ggfs. durch die CRC Prüfung und ResendAnforderung abgedeckt)
- Encryption auf AES Basis (abgeleitet von der LMIC AES Lib)
- Multi Band management (heute wird nur 1 band gewählt, das aber nur alle 10-15 Minuten benutzt)
- Duty Time recognition

Hauptanbieter des LoRa-MAC Layer HW Moduls ist die Firma Semtech, die auch die **[LoRaWAN Spezifikation v1.0.3](https://lora-alliance.org/resource-hub/lorawanr-specification-v103)** als Member der "LoRA Alliance" mit herausgegeben hat.
Die Firma Dragino hat auf Basis dieses Quasi-Standard Modules (basierend auf dem SX1276/SX1278 transceiver chips) diverse Hats & Shields entworfen.
Der kleinste Vertreter davon (ohne GPS Modul) ist das "Dragino Lora-Bee Modul" **[(Wiki)](http://wiki.dragino.com/index.php?title=Lora_BEE)**, welches via SPI angeschlossen wird.

Darauf befindet sich ein RFII95-98W mit SPI Interface. Dieses Basismodul von Semtech kann man aber auch günstig (2-6€) in Asien bestellen) und erfüllen densleben Zweck. Die Draginomodule nehmen einem nur zusätzliche Verdrahtung und ggfs. den Antennenanschluss ab.

Die Verdrahtung ist recht einfach:
Neben den Standard shared (!) SPI Leitungen (MISO, MOSI, SCK) gibt es noch die Modul-spezifische CS Leitung zur Modul-Selektion, eine Reset-Leitung (RST) und 6 universelle Daten-IO Leitungen für weitere Funktionen DIO0..DIO5. 
DIO0 bildet z.B. den LoRa Interrupt ab und triggert bei RX/TX-Events. Für den Standard LoRa Betrieb werden die übrigen DIO1-DIO5 Leitung aber i.d.R. nicht benötigt (solange man beim LoRa-Mode bleibt; Für den FSK Mode werden häufig auch DIO0-2 genutzt).
Daher habe ich in dieser Schaltung nur DIO0 + DIO1 auf duplex fähige GPIO Leitungen mappen können, und DIO2 auf eine Read Only Leitung (weil sie noch frei war, aber geshared mit dem Key3 des ePaper Moduls; aber aktuell ohne Funktion bleibt). Ggfs. kann man darüber noch einen manuellen Sendetrigger imlementieren. 
Alle 3 Leitungen werden aber nur im Input Mode betrieben (zur Signalisierung des Semtech Modul Status).

<img src="./images_v2/Dragino_Lora_Bee.jpg"> <img src="./images_v2/Dragino_Lora_Bee_Cabling.jpg">

Auf dem 2. Bild ist das grün gefärbte Semtech LoRa Modul gut zu erkennen.

Die Spezifikation weisst folgende Eigenschaften aus:
- 168 dB maximum link budget.
- +20 dBm - 100 mW constant RF output vs.
- +14 dBm high efficiency PA.
- Programmable bit rate up to 300 kbps.
- High sensitivity: down to -148 dBm.
- Bullet-proof front end: IIP3 = -12.5 dBm.
- Excellent blocking immunity.
- Low RX current of 10.3 mA, 200 nA register retention.
- Fully integrated synthesizer with a resolution of 61 Hz.
- FSK, GFSK, MSK, GMSK, LoRaTM and OOK modulation.
- Built-in bit synchronizer for clock recovery.
- Preamble detection.
- 127 dB Dynamic Range RSSI.
- Automatic RF Sense and CAD with ultra-fast AFC.
- Packet engine up to 256 bytes with CRC.
- Built-in temperature sensor and low battery indicator.

Das Dragino Manual dazu findet sich **[hier](http://wiki.dragino.com/index.php?title=Lora_BEE)**.

<img src="./images_v2/Duck_Antenna.jpg">
Da aber nahezu alle Leitungen des SemTech Moduls 1.1 am Bee-Sockel ausgeführt sind, kann man im Grunde jede Bibliothek verwenden, die den SX1276 (für 868MHz) unterstützt.

Die aktuell verwendeten GPIO Port Definitionen:
```cpp
#include <SPI.h>	// default for all SPI devices
// Libraries for LoRa
#include "LoRa.h"

// LoRa-Bee Board at VSPI port
#define BEE_MISO VSPI_MISO	// SPI MISO -> VSPI
#define BEE_MOSI VSPI_MOSI	// SPI MOSI -> VSPI
#define BEE_SCK  VSPI_SCK	// SPI SCLK -> VSPI
#define BEE_CS   	12		// NSS == CS
#define BEE_RST		14  	// Reset\
#define BEE_DIO0	33		// Main Lora_Interrupt line
#define BEE_DIO1	13		// for Bee-Events
#define BEE_DIO2	34		// unused by BEE_Lora;  connected to EPD K3 -> but is a RD only GPIO !
```
Die Rolle des Client Node MAC layers ist in dieser **[Backend Specification](https://lora-alliance.org/resource-hub/lorawanr-back-end-interfaces-v10)** festgehalten
Aktuell gehen meine Tests, wie oben bereits erwähnt, in Richtung der LMIC LoRaWAN SW Stack Implementierung von IBM. Der MAC Layer ist durch die Implementierungen in hal.c + Radio.c enthalten.
(siehe GitHub -> "lmic_pi"). Dieses habe ich als Basis für den GW seitigen BeeIot-WAN Aufbau auf Basis RPi verwendet.
Alternative sei auch noch die "RadioHead" Implementierng genannt (s. GitHub).

Für ESP32 MAC Layer Testzwecke habe ich im Sketch aktuell die Lora-Library von Sandeep (GitHub) in Verwendung. Diese ist für eine stabile MAC Layer Kommunikation vollkommen ausreichend. Über eine vollständige WAN Kommunikation kümmere ich mich später...

Da ich ein eigens "BeeIoT-"LoRaWAN Netzwerk aufbauen möchte ist die Rolle dieses ESpP32 Nodes: Activation-by-Personalization (ABP) und als eindeutige statisceh LoRa Devide-ID habe ich daher die ESP32 interne BoardID (basierend auf der WiFi MAC Adresse) vorgesehen. Daraus kann auch die LoRa-"DevAddr" zur Node-Protokoll ID (DevID) gebildet werden.
Eine Anbindung an das allseits beliebte offene TT-Netzwerk schränkt die Nutzung durch limitierte Anzahl Daten und Paket/Zeitraum zu sehr ein.
Als Advanced feature ist OTAA anzusehen. Der standardisierte Weg über LoRaWAn FW updates remote auf den ESP32 als Lora Node aufzuspielen. Damit ist echte Fernwartung möglich, (wie sonst nur bei einer echten LAN Verbindung und laufendem OS auf dem Node wie bei einem RPi.) Für solch intensiven Datenaustausch ist aber Multiband management mit Duty Time Control nötig, weil sonst das einzelne Band zu sehr vereinnahmt wird.

#### LoRa WAN Support - Gateway side

```
// lflags = LOGBH + LOGOW + LOGHX + LOGLAN + LOGEPD + LOGSD + LOGADS + LOGSPI + LOGLORA;
lflags = LOGBH + LOGLORA + LOGLAN;
```
Darüber kann man nach Belieben verschiedene Funktionsbereiche in den verbose mode schalten und analysieren, so dass der Log-Output nicht von unnötigen Meldungen überschwemmt wird.

Mit den obigen 3 Schalter: LOGBH + LOGLORA + LOGLAN
könnte der Konsol-Output wie folgt aussehen:








Das war es soweit erstmal von meiner Seite.

Viel Spass damit und einen Imkerlichen Gruss
wünscht Euch 

Randolph Esser
(mail(a)RandolphEsser.de)


[**www.RandolphEsser.de**](http://www.RandolphEsser.de/Imkerei)
==> Imkerei
