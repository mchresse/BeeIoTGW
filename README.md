# BeeIoTGW
LoRa based Gateway and Networkserver for BeeIoT Nodes (on Raspberry Pi + Dragino LoRa Hat).

The BeeIoTWAN Network corresponds to the project: mchresse/BeeIoT and is derived from LoRaWAN.

### License:
See the **[LICENSE](https://github.com/mchresse/BeeIoTGW/blob/master/LICENSE)** file for license rights and limitations (New BSD-clause).

==============================
This is the corresponding Raspberry Pi based LoRa Gateway and Network service for the BeeIoT Nodes (as described in the mchresse/BeeIoT project).

Both projects are using the proprietary LoRa BIoTWAN network protocol optimized for serving multiple BeeIoT Nodes (based on ESP32).
The Radio layer code supports all LoRa hats with a Semtech SX1276 chip only and also in LoRa Mode only (no FSK modulation support).


###Features
--------
Each BeeIot Node contacts the gateway by a LoRa JOIN packet providing a node individual devID.
Each DevID can be preregistered gateway side for join request evaluation.
As response to the join request the cfg-ID as index from a shared configurable channeltable allows several different Lora modem config sets to be selected for the current protocol session.
But as long as the gateway is single channel only from physical chip side (one SX chip is used), the selected channel cfg. set has to be used for all registered nodes to ease up the gateway code and allows further join requests also. The typical bandwidth 125kHz on EU868MHz Frequency.
All spreadings SF7 - Sf12 are possible.
The default config channel set is: 868.1MHz, BW125, SF7, CR4_5.

After successful registration, each nodes can report sensor data packs frequently to the gateway, evaluated by the network server and join server instances and final forwarded to the BIoT application layer.
The final validated sensor data is stored in a CSV file at /var/www/html/beeiot/beeiot<year>.csv file for further processing by a html file for web presentation (e.g. with DyGraph).

####Planned features: 
- Read out SD card data from node to gateway.
- Download FW updates from gateway to node.
- individualize different CSV files by NodeID.

####HW-Dependencies
------------
Prerequisites on Raspberry Pi:
- install WiringPi library and link the project with -lwiringpi flag. 
- SPI protocol must be enabled by RPI-config tool
- Start program as root
