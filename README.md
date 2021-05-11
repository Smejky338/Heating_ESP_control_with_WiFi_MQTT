# Heating radiator valve with WiFi+MQTT
 Modification of thermostatic radiator valve. Using ESP8266 for communication via WiFi+MQTT with base station (eg. running Home Assistant)

Modified cheap heating radiator valve (TRV) head Eqiva EQ3-N to control heating logic using PID controller.
Also added connected other functionality thanks to ESP8266-based NodeMCU D1 mini: WiFi & MQTT connection to central control unit such as Home Assistant, window opened detection, boost mode control from Home Assistant by pressing a button on the TRV head, light sensor, temperature sending to log in Home Assistant.

If there are invalid credentials, the head starts WiFi Access Point where you can set new credentials via integrated config webpage.

made for Arduino IDE.

![hlavice-nainstalovana](https://user-images.githubusercontent.com/16916837/117783437-d2411b80-b242-11eb-93d1-e63aa802ada7.jpg)

Config webpage:
![ESP-aktualizace-údajů](https://user-images.githubusercontent.com/16916837/117785266-b3438900-b244-11eb-924d-e8531e845c0f.png)
