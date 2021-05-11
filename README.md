## Heating radiator valve with WiFi+MQTT

 Modification of thermostatic radiator valve. Using ESP8266-based NodeMCU D1 mini for communication via WiFi+MQTT with base station (eg. running Home Assistant)

Modified cheap heating radiator valve (TRV) head Eqiva EQ3-N to control heating logic using PID controller (likely will need adjustment of ki,kp,kd variables). More about used PonM PID controller at http://brettbeauregard.com/blog/2017/06/introducing-proportional-on-measurement/.
Also added connected other functionality thanks to NodeMCU D1 mini: WiFi & MQTT connection to central control unit such as Home Assistant, window opened detection, boost mode control from Home Assistant by pressing a button on the TRV head, light sensor, temperature sending to log in Home Assistant.

If there are invalid credentials, the head starts WiFi Access Point where you can set new credentials via integrated config webpage.

![hlavice-nainstalovana](https://user-images.githubusercontent.com/16916837/117783437-d2411b80-b242-11eb-93d1-e63aa802ada7.jpg)

### Files
- *Heating_ESP_control.ino* : source code for Arduino IDE, in order to upload compiled sketch to NodeMCU D1 mini you'll likely need to install drivers for CH340 USB-Serial converter from https://docs.wemos.cc/en/latest/ch340_driver.html.
- *wiring/* : folder with wiring schematics and pictures of modified valve
- *pictures/* : folder with pictures of fully assembled complete head unit installed on radiator
- *Home Assistant config/* : folder with config file to connect Home Assistant to MQTT broker and to inegrate devices, next file is source code of thermostat program made in Node-RED (*ThermostatNodeRED.json*, available from https://gist.github.com/giuseppeg88/25857cba1ab7e34309133ac8354004c6).

Config webpage:
![ESP-aktualizace-údajů](https://user-images.githubusercontent.com/16916837/117785266-b3438900-b244-11eb-924d-e8531e845c0f.png)

This project was made for my bachelor thesis at FIT BUT Brno, Czech description below:

## Hlavice pro vytápění s WiFi+MQTT: Přírůčka k bakalářské práci

název: Digitální termostatická hlavice řízená přes WiFi
Jan Smejkal, xsmejk27
FIT VUT Brno
květen 2021

V práci je upravena hlavice Eqiva EQ3-N, přidán modul NodeMCU D1 mini a pomocí WiFi a MQTT připojen do systému chytré domácnosti Home Assistant.

### Soubory
Samotné soubory a jejich obsah včetně komentářů jsou v anglickém jazyce.

- *Heating_ESP_control.ino* : soubor se zdrojovým kódem hlavice pro Arduino IDE, pro nahrání na NodeMCU D1 mini bude třeba nainstalovat ovladače pro převodník CH340, například z https://docs.wemos.cc/en/latest/ch340_driver.html.
- *wiring/* : složka se schématem a obrázky úpravy
- *pictures/* : složka fotografií nainstalované hlavice
- *Home Assistant config/* : složka s konfiguračním souborem pro připojení Home Assistanta k MQTT brokeru a integrací zařízení, dále zdrojový kód programu termostatu pro Node-RED (*ThermostatNodeRED.json*).
- *poster/poster.pdf* : PDF plakátu k bakalářské práci

