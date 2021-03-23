/*
   TODO:
   switch PID control and tune params (hard because I part stacks up when central heating is off)
      - test & tune
   Opens AP to login and change credentials of network & MQTT broker 
      - done, possible extension: show list of available WiFi networks
        ^https://github.com/esp8266/Arduino/blob/master/libraries/ESP8266WiFi/examples/WiFiScan/WiFiScan.i
*/
#include <ESP8266WiFi.h>

//HTTP server:
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>

#include <string.h>

//to store and load config of WiFi+MQTT to connect to in flash
#include <FS.h>
#include <ArduinoJson.h>

//for OTA updates
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

//time for decalc every saturday
#include <NTPClient.h>

//DS18B20 reading using lib by Rob Tillaart
#include <DS18B20.h>
#include <OneWire.h>

//PID
#include "QuickPID.h"

//MQTT:
#include <PubSubClient.h>

//default credentials of network, optional - example inputs are in #IFNDEF below, can be also changed via web interface:
//just connect to the AP (using APSSID + APPSK) and enter 192.168.4.1 or SSID_OF_AP + ".local"
#include "credentials.h"

//temperature reading:
const int DSpin = D5;
OneWire oneWire(DSpin);
DS18B20 sensor(&oneWire);

//maximum number of letters - 1 for each credential stored in flash
#define SIZE_CREDENTIAL_UNIT 25

//credentials can be included in credentials.h, example entries below:
//WiFi client (STAtion) fallback credentials
#ifndef STASSID
#define STASSID "WiFiSSID"
#define STAPSK  "WiFipassword"
#endif

//WiFi host (AP) fallback credentials
#ifndef APSSID
#define APSSID "ESPValve"
#define APPSK "WiFiAPpassword"
#endif

//MQTT fallback credentials, should be included from credentials.h
#ifndef MQTTCRED
const char *mqttServer = "192.168.X.XXX"; //max len of ipv6 addr (just in case someone switches to IPv6, this project expects IPv4
const int  mqttPort = 1883;
const char *mqttUser = "ESPclient";
const char *mqttPassword = "mqttpassword";

//password for uploading OTA updates to ESP
const char* OTApassword = "otapassword";
#endif

/**
 * Struct for loading/getting config to store it in flash in FS
 */
struct Config {
  char ap_ssid [SIZE_CREDENTIAL_UNIT];
  char ap_psk [SIZE_CREDENTIAL_UNIT];
  char mqtt_server [40];
  char mqtt_port [SIZE_CREDENTIAL_UNIT];
  char mqtt_user [SIZE_CREDENTIAL_UNIT];
  char mqtt_password [SIZE_CREDENTIAL_UNIT];
};

struct Config *loaded_config;

//pins to control H-bridge and in turn control the valve motor:
const int T1 = D7;
const int T3 = D8;

//stores last direction of motor
#define DIR_OPEN 1
#define DIR_CLOSE 2
#define DIR_NONE 0
int dir = DIR_NONE;

//stores sum of time that motor moved in series in one direction
// so in summer it doesn't just close all the time
unsigned long cnt_same_direction = 0;
const unsigned long same_direction_threshold = 60 * 1000;
unsigned long time_until = 0;

#define T_SIZE 6
//recent temperatures from last measurements
double temps[T_SIZE] = {21, 21, 21, 21, 21, 21};

//determines how often in ms should the valve recalculate and react to temperature change
#define REACT_TIME 60 * 1000

unsigned long time_last_react;

//hysteresis when valve reacts on temperature over/under desired temp. target
#define HYSTERESIS_OVER 0.05

#define HYSTERESIS_UNDER 0.2

#define DIFF_MULTIPLIER 4

//multiplier for closing/opening valve in ms per 1 dg. C
#define CLOSE_MULTIPLIER 5000

#define OPEN_MULTIPLIER 1500

//multiplier for closing valve when already above desired temp so it doesn't overshoot
#define OPEN_ABOVE_MULTIPLIER 200

double target_temp = 21.0;
bool heating_on = true;

/**PID part:
  *using library available from https://github.com/Dlloydev/QuickPID
*/
int16_t Setpoint = target_temp, Input, Output;

float aggKp = 2, aggKi = 0.01, aggKd = 0.1;
float consKp = 10, consKi = 0.0001, consKd = 0.00;
float aggPOn = 0.1; // Range is 0.0 to 1.0 (1.0 is 100% P on Error, 0% P on Measurement)
float consPOn = 0.00; // Range is 0.0 to 1.0 (0.0 is 0% P on Error, 100% P on Measurement)

/**Stores previous position of valve, used to determine motor movement
  *0 = fully closed valve, 255 = open
*/
int8_t prev_position;

//initial tuning parameters:
QuickPID myQuickPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, aggPOn, DIRECT);

//window opened/closed detection:
bool window_opened = false;
#define HYSTERESIS_OPENED_WINDOW 1
#define HYSTERESIS_CLOSED_WINDOW 0.3

//NTP section, updates every 48h.
#define NTP_PERIOD 1000*60*60*48

//local offset in s. 1hr=3600s
#define PragueOffset 3600 //UTC+1

WiFiUDP ntpUDP;
//parameters:   ptr WiFiUDP, serveraddr           , offset[s], update interval [ms]
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 3600, NTP_PERIOD);

unsigned long last_ntp = 0;

bool decalc_done = false;

//MQTT
//period between sending current temperature
#define MQTT_SEND_PERIOD 30*1000

unsigned long time_last_send = 0;

WiFiClient espClient;
PubSubClient client(espClient);

//HTTP server inspired from example AdvancedWebServer Copyright (c) 2015, Majenko Technologies
ESP8266WebServer server(80);

/**
   HTTP webpage inspired from user ACROBOTIC at https://www.youtube.com/watch?v=lyoBWH92svk 
   and w3schools at https://www.w3schools.com/css/tryit.asp?filename=trycss_forms
*/
char webpage[] PROGMEM = R"=====(
<html>
<head>
  <style>
    body {
      font-family: 'Segoe UI', sans-serif;
    }
    input {
      width: 100%;
      padding: 12px 20px;
      margin: 8px 0;
      display: inline-block;
      border: 1px solid #ccc;
      border-radius: 4px;
      box-sizing: border-box;
    }
    
    button {
      width: 100%;
      background-color: #4CAF50;
      color: white;
      padding: 14px 20px;
      margin: 8px 0;
      border: none;
      border-radius: 4px;
      cursor: pointer;
    }
    
    button :hover {
      background-color: #45a049;
    }
    
    form {
      border-radius: 5px;
      background-color: #f2f2f2;
      padding: 20px;
    }
  </style>
</head>
<body>
<h2>ESP Heating Valve Credentials Update</h2>
<form>
  <label for="WIFIssid">WiFi SSID</label>
  <input value="" id="WIFIssid" placeholder="SSID of WiFi to connect to"/>

  <label for="WIFIpassword">WiFi password</label>
  <input value="" id="WIFIpassword" placeholder="password for WiFi to connect to"/>

  <label for="MQTTip">MQTT IP address</label>
  <input value="" id="MQTTip" placeholder="IP address of MQTT broker"/>

  <label for="MQTTport">MQTT port</label>
  <input value="" id="MQTTport" placeholder="port of MQTT broker" value="1883"/>

  <label for="MQTTusername">MQTT username (optional)</label>
  <input value="" id="MQTTusername" placeholder="MQTT username"/>

  <label for="MQTTpassword">MQTT password (optional)</label>
  <input value="" id="MQTTpassword" placeholder="MQTT password"/>

  <button onclick="save()">Save changes</button>
</form>
</body>
<script>
  function save() {
    var ap_ssid = document.getElementById("WIFIssid").value;
    var ap_psk = document.getElementById("WIFIpassword").value;
    var mqtt_server = document.getElementById("MQTTip").value;
    var mqtt_port = document.getElementById("MQTTport").value;
    var mqtt_user = document.getElementById("MQTTusername").value;
    var mqtt_password = document.getElementById("MQTTpassword").value;
    var data = {ap_ssid:ap_ssid, ap_psk:ap_psk, mqtt_server:mqtt_server, mqtt_port:mqtt_port, mqtt_user:mqtt_user, mqtt_password:mqtt_password};

    var xhr = new XMLHttpRequest();
    var url = "/settings"; 

    xhr.onreadystatechange = function(){
      if (this.onreadyState == 4 && this.status == 200) {
        console.log(xhr.responseText);
      }
    };
    xhr.open("POST", url, true);
    xhr.send(JSON.stringify(data));
  }
</script>

</html>
)=====";

/**
 * Shows root webpage with config:
 *  - WiFi SSID+password
 *  - MQTT IP address
 *  could show list of APs nearby
 */
void HandleRoot(){
  server.send_P(200, "text/html", webpage);
}

/**
 * stores JSON data to flashsfile system
 */
void HandleSettingsUpdate(){
  //receive JSON data to data and parse it
  String data = server.arg("plain");
  DynamicJsonBuffer j_buffer;
  JsonObject& j_object = j_buffer.parseObject(data);

  //store parsed data to file
  File config_file = SPIFFS.open("/config.json", "w");
  if (!config_file){
    Serial.println("ERR opening /config.json in HandleSettingsUpdate!!!");
    return;
  }
  j_object.printTo(config_file);
  config_file.flush();
  config_file.close();

  server.send(200, "application/json", "{\"status\":\"ok\"}");
  
  Serial.println("Settings updated, Restarting in 2s ...");

  delay(2000);
  
  ESP.restart();
}

/**
 * Starts HTTP servers listening for incoming requests.
 */
void InitWeb(){
  server.on("/", HandleRoot);
  server.on("/settings", HTTP_POST, HandleSettingsUpdate);
  server.begin();
  Serial.println("HTTP server started");
}

/**
 * Loads config and stores it in variable
 * inspired from ConfigFile example by Ivan Grokhotkov.
 */
struct Config *LoadConfig(const char *filename){
  File config_file = SPIFFS.open("/config.json", "r");
  if (config_file) {
    struct Config *conf_ret = (struct Config*) malloc((sizeof(struct Config)));
    size_t size = config_file.size();
    
    std::unique_ptr<char[]> buf(new char[size]);
    config_file.readBytes(buf.get(), size);

    DynamicJsonBuffer j_buffer(200);
    JsonObject &j_object = j_buffer.parseObject(buf.get());
    if (j_object.success()){
      strncpy(conf_ret->ap_ssid,       j_object["ap_ssid"],      SIZE_CREDENTIAL_UNIT-1);
      conf_ret->ap_ssid[SIZE_CREDENTIAL_UNIT-1] = '\0';
      strncpy(conf_ret->ap_psk,        j_object["ap_psk"],       SIZE_CREDENTIAL_UNIT-1);
      conf_ret->ap_psk[SIZE_CREDENTIAL_UNIT-1] = '\0';
      strncpy(conf_ret->mqtt_server,   j_object["mqtt_server"],  39);
      conf_ret->mqtt_server[39] = '\0';
      strncpy(conf_ret->mqtt_port,     j_object["mqtt_port"],    SIZE_CREDENTIAL_UNIT-1);
      conf_ret->mqtt_port[SIZE_CREDENTIAL_UNIT-1] = '\0';
      strncpy(conf_ret->mqtt_user,     j_object["mqtt_user"],    SIZE_CREDENTIAL_UNIT-1);
      conf_ret->mqtt_user[SIZE_CREDENTIAL_UNIT-1] = '\0';
      strncpy(conf_ret->mqtt_password, j_object["mqtt_password"], SIZE_CREDENTIAL_UNIT-1);
      conf_ret->mqtt_password[SIZE_CREDENTIAL_UNIT-1] = '\0';

      Serial.println("Succesfully loaded json data from FS.");
      
      return conf_ret;
    } else {
      Serial.println("Failed to parse config data.");
      return NULL;
    }
  } else {
    Serial.println("Failed to open config file");
    return NULL;
  }
}

/**
 * Initializes WiFi connection and starts endpoint for OTA updates. 
 */
bool WifiOtaON(){
  //default is STATION+AP and we want that
  //to configure WiFi credentials + IP address of server
  //WiFi.mode(WIFI_STA);
  
  //start host/AP:
  //firstly let's add random string to differentiate from other ESPValves around here:
  char APssid[30] = APSSID;
  if (strnlen(APssid, 25) < 24){
    strcat(APssid, "-");
    strcat(APssid, String(random(0xffff), HEX).c_str());
  } else {
    APssid[29] = '\0';
  }
  //the AP start itself:
  if (WiFi.softAP(APssid, APPSK)){
    Serial.print("AP IP addr:");
    Serial.println(WiFi.softAPIP());
  }

  //secondly, start station/client:
  if (loaded_config != NULL){
    WiFi.begin(loaded_config->ap_ssid, loaded_config->ap_psk);
  } else {
    WiFi.begin(STASSID, STAPSK);  
  }
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Connection Failed!");
  }
 
  if (!MDNS.begin(APssid)){
    Serial.println("mDNS startup failed!");
  } else {
    Serial.print("MDNS responder started at name=");
    Serial.println(APssid);
  }
  
  //set password for OTA uploading new code to ESPs
  ArduinoOTA.setPassword(OTApassword);

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS unmount it using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

/**
 * handles incoming MQTT messages.
*/
void handleMQTT(char* topic, byte* payload, unsigned int length){
  char *pomtext = (char*) malloc( sizeof(char) * ( length + 1 ) );

  Serial.print("\nMessage arrived, topic: ");
  Serial.println(topic);
  
  if (pomtext == NULL){
    Serial.print("ERR MQTT text could not be allocated, discarding, length=");
    Serial.println(length);
    return;
  }
  
  unsigned i = 0;
  for (; i<length; i++){
    pomtext[i] = payload[i];
  }
  pomtext[i] = 0;

  //determine which topic arrived and proccess incoming value:
  if (!strcmp(topic, "living_room/valve1/desired_temp/set")){
    target_temp = atof(pomtext);

    Setpoint = target_temp;
    Serial.print("New target_temp=");
    Serial.println(target_temp);
  } else if (!strcmp(topic, "living_room/valve1/mode/set")){//off|heat
    if (strncmp(pomtext, "heat", 4) == 0){
      heating_on = true;
      Serial.println("heat on");
    } else if (strncmp(pomtext, "off", 3) == 0){
      heating_on = false;
      Serial.println("heat off");
    }
  } else {
    Serial.print("Incoming MQTT msg discarded, topic=");
    Serial.println(topic);
  }
}

/**
 * initializates MQTT connection. Subscribing happens in setup()
 * inspired from https://techtutorialsx.com/2017/04/09/esp8266-connecting-to-mqtt-broker/
 */
bool MQTTinit(){
  char m_server[40], m_user[SIZE_CREDENTIAL_UNIT], m_password[SIZE_CREDENTIAL_UNIT];
  int m_port;
  if (loaded_config != NULL){
    Serial.println("Loading MQTT config from json in FS:");
    strncpy(m_server, loaded_config->mqtt_server, 39);
    m_server[39] = '\0';
    
    m_port = atoi(loaded_config->mqtt_port);
    strncpy(m_user, loaded_config->mqtt_user, SIZE_CREDENTIAL_UNIT-1);
    m_user[SIZE_CREDENTIAL_UNIT-1] = '\0';
    strncpy(m_password, loaded_config->mqtt_password, SIZE_CREDENTIAL_UNIT-1);
    m_password[SIZE_CREDENTIAL_UNIT-1] = '\0';
  } else {
    Serial.println("Loading default MQTT config");
    strcat(m_server, mqttServer);
    m_port = mqttPort;
    strcpy(m_user, mqttUser);
    strcpy(m_password, mqttPassword);
  }
  char mqttID[20]="ESPvalve-";
  strcat(mqttID, String(random(0xffff), HEX).c_str());
  
  client.setServer(m_server, m_port);
  client.setCallback(handleMQTT);
  unsigned cnt = 0;
  
  if (client.connect(mqttID, m_user, m_password )) {
    Serial.println("MQTT connected");  
    
    //SUB to topics valve is interested in to get info about for its operation (controls, weather info, ...)
    client.subscribe("living_room/valve1/desired_temp/set");
    client.subscribe("living_room/valve1/mode/set");
    
    //MQTT publish info window closed (default when just started)
    client.publish("living_room/valve1/window_opened", "OFF");
    
    return true;
  } else {
    Serial.print("failed with state ");
    Serial.println(client.state());
    return false;
  }
}

/**
 * gets temperature from DS temperature sensor.
 * returns temperature as double.
 */
double getTemp() {
  sensor.requestTemperatures();
  
  while (!sensor.isConversionComplete()){
    // wait until sensor is ready, yield to ESP for now
    yield();
  }

  return sensor.getTempC();
}

/**
 * Pushes back incoming temperature newtemp to the end of array, moving others to the left.
 * parameter double newtemp - incoming temperature
 */
void TempsPushBack(double newtemp){
  unsigned i = 0;
  for (; i < T_SIZE - 1 ; i++){
    temps[i] = temps[i+1];
  }
  temps[i] = newtemp;
}

/**
 * Calculates possible future temperature
 * Returns predicted future temperature based on previous 4 measurements
 * just an average of simple linear extrapolations 
 * not used by PID
 */
double PredictTemp(){
  double sum = 0;
  for(unsigned i = 0; i < T_SIZE - 1; i++){
    sum += temps[i+1] - temps[i];
  }
  return temps[T_SIZE-1] + sum / (T_SIZE-1);
}

/** 
 * Returns true if temperature drop is over hysteresis
 * Excepts T_SIZE to be at least 6, MINIMUM is 4 for meaningful results or error
 */
bool OpenedWindow(){
  if (T_SIZE < 4){
    //failsafe
    Serial.println("ERR: function OpenedWindow could not be correctly called, T_SIZE has to be >= 4");
    return false;
  }
  if (window_opened == false){
    if ( ((temps[0] + temps[1]) / 2) - ((temps[T_SIZE-2] + temps[T_SIZE-1]) / 2) >= HYSTERESIS_OPENED_WINDOW ){
      return true;
    }
  //if window is already detected open, check if closed (last measurement is rising against previous two)
  } else if ( ((temps[T_SIZE-2] + temps[T_SIZE-1]) / 2) - ((temps[T_SIZE-4] + temps[T_SIZE-3]) / 2) >= HYSTERESIS_CLOSED_WINDOW ){
    return false;
  } else {
    return true;
  }
  return false;
}

/**
 * Checks if it's time for decalc and if yes, then it does it.
 * By default it decals every saturday between 11 and 12.
 */
bool Decalc(){
  //getDay returns 0-6 starting Sunday
  if (timeClient.getDay() == 6 && timeClient.getHours() == 11 && decalc_done == false){
    decalc_done = true;
    MotorOpen(same_direction_threshold);
    delay(same_direction_threshold);
    MotorClose(same_direction_threshold);
    
    return true;
  } else if (timeClient.getDay() == 6 && timeClient.getHours() > 12) {
    decalc_done = false;
  }
  return false;
}

/**
 * Opens motor and sets var time_until declaring how long should it be opening for.
 * param time time in ms to open for. The time is in reality likely to be a few hundreds of ms longer.
 */
void MotorOpen(unsigned time){
  if (dir == DIR_OPEN && cnt_same_direction >= same_direction_threshold) {
    Serial.println("NOT opening valve, over threshold, cnt=" + String(cnt_same_direction));
    return;
  }
  digitalWrite(T1, HIGH);
  digitalWrite(T3, LOW);
  
  if (dir == DIR_OPEN){
    cnt_same_direction += time;
  } else {
    dir = DIR_OPEN;
    cnt_same_direction = time;
  }
  
  time_until = millis()+time;
  Serial.println("Opening valve, should be opened in " + String(time) + "ms at " + String(time_until) + ".");
}

/**
 * Closes motor and sets var time_until declaring how long should it be closing for.
 * param time time in ms to close for. The time is in reality likely to be a few hundreds of ms longer.
 */
void MotorClose(unsigned time){
  if (dir == DIR_CLOSE && cnt_same_direction >= same_direction_threshold) {
    Serial.println("NOT closing valve, over threshold, cnt=" + String(cnt_same_direction));
    return;
  }
  digitalWrite(T1, LOW);
  digitalWrite(T3, HIGH);

  if (dir == DIR_CLOSE){
    cnt_same_direction += time;
  } else {
    dir = DIR_CLOSE;
    cnt_same_direction = time;
  }

  time_until = millis()+time;
  Serial.println("Closing valve, should be closed in " + String(time) + "ms at " + String(time_until) + ".");
}

/*
 * Checks if Open/Close happened for desired time and should be stopped.
 * returns true when canceling action, false when no action canceled.
*/
bool CheckMotorTimeUntil(){
  if (time_until <= millis()-100){
    digitalWrite(T1, HIGH);
    digitalWrite(T3, HIGH);
    delay(100);
    if (time_until > 0) {
      Serial.print("Cancelling closing/opening, should have been closed/opened at T_U=" + String(time_until) + ", diff=");
      Serial.println(String(millis()-time_until) + ", now=" + String(millis()));
    }
    time_until = 0;
    
    return true;
  }
  return false;
}

/**
 * Initializes all used parts of program.
 * Tries to load config, tries connecting to WiFi, MQTT, 
 *  starts webserver, mDNS responder, PID, resets valve state to open.
 */
void setup() {
  // initialize GPIO pins:
  pinMode(T1, OUTPUT);
  pinMode(T3, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(LED_BUILTIN, LOW);

  //initializes serial communication, accesible by cable:
  Serial.begin(115200);
  Serial.println("Booting");

  SPIFFS.begin();
  
  loaded_config = LoadConfig("/config.json");

  //starts WiFi AP+STAtion and OTA updates endpoint and mDNS responder
  WifiOtaON();
  
  //initialize sensor and measure first temp:
  sensor.begin();
  temps[0] = getTemp();
  for (int i=1; i<T_SIZE; i++){
    temps[i] = temps[i-1];
  }
  
  timeClient.begin();
  timeClient.update();

  MQTTinit();

  InitWeb();
  
  Input = temps[T_SIZE-1];
  Setpoint = target_temp;
  
  //turn the PID on
  myQuickPID.SetMode(AUTOMATIC);

  delay(2000);//wait for the original board to do its bootup ...

  MotorOpen(same_direction_threshold);
  prev_position = 255;

  time_last_react = 0;
}

/**
 * Main loop of the program.
 * If there is problem connecting to WiFI or MQTT, the LED is ON.
 */
void loop() {
  char temps_text[32];
  double temp;
  bool gotTemp = false;
  int8_t diff_position;

  //Handle possible OTA updates:
  ArduinoOTA.handle();

  //check if it's time to stop motor movement:
  CheckMotorTimeUntil();

  //try to connect to WIFI if not connected
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Trying to reconnect to WiFi..");
    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
      Serial.println("WiFi Connection Failed!");
      digitalWrite(LED_BUILTIN, LOW);//LED ON
    }
  //if connected to WiFi, try to init MQTT connection if not connected to MQTT:
  } else if (!client.connected()){
    digitalWrite(LED_BUILTIN, LOW);//LED ON
    MQTTinit();
  } else {
    digitalWrite(LED_BUILTIN, HIGH);//LED OFF
    //keep MQTT connection and get possible messages with updates
    client.loop();
  }
  
  server.handleClient();
  MDNS.update();
  
  Decalc();

  //called twice inside this loop so there is not that long possible delay
  CheckMotorTimeUntil();
  
  //check if it is time to get temp and determine motor movement:
  if (millis() - time_last_react > REACT_TIME){
    gotTemp = true;
    temp = getTemp();
    TempsPushBack(temp);
    
    sprintf(temps_text, "Temps: %f %f %f %f", temps[0], temps[1], temps[2], temps[3]); 
    Serial.println(temps_text);

    /**
     * Experimental PID part
    */
    Input = temps[T_SIZE-1];
    if ((Input - target_temp) > -0.5) { //we're close or above setpoint, use conservative tuning parameters
      myQuickPID.SetTunings(consKp, consKi, consKd, consPOn);
    } else {
      //we're far from setpoint, use aggressive tuning parameters
      myQuickPID.SetTunings(aggKp, aggKi, aggKd, aggPOn);
    }
        
    myQuickPID.Compute();
    
    //check if heating is on/off and no motor movement is happening:
    if (time_until == 0 && heating_on == true){
      //check opened window => close valve
      if (OpenedWindow() == true){
        window_opened = true;
        MotorClose(same_direction_threshold);
        prev_position = 0;
        
        //MQTT publish info about opened window
        client.publish("living_room/valve1/window_opened", "ON");
        //if window was open and is now closed
      } else if (window_opened == true){
        window_opened = false;
        //MQTT publish info window closed
        client.publish("living_room/valve1/window_opened", "OFF");
        //if temp is too low, continue heating from full blast:
        if (temps[T_SIZE-1] + 3 < target_temp){
          MotorOpen(same_direction_threshold);
          prev_position = 255;
        }
      //heating is ON and window is closed, so we can control heating:
      } else {
        diff_position = Output - prev_position;
        //Check if diff of new position is a bit significant:
        if (abs(diff_position) >= 5){
          //move in the correct direction:
          if (diff_position > 0){
            MotorOpen(abs(diff_position) / 255.0 * same_direction_threshold);
          } else {
            MotorClose(abs(diff_position) / 255.0 * same_direction_threshold);
          }
          prev_position = Output;
        }
        
        char outputstr[16];//just to be sure in case int somehow becomes bigger than 16b
        sprintf(outputstr, "%d", Output);
        client.publish("living_room/valve1/PIDvalue", outputstr);
      }
    } else if (heating_on == false){
      MotorClose(same_direction_threshold);
    }
    
    //log that reaction happened:
    time_last_react = millis();
    /*
     * End of PID part
    */

    /*
    //if no motor movement is happening and heating is on, determine if there should be some:
    if (time_until == 0 && heating_on == true){
      double predict_temp = PredictTemp();
      Serial.print("predict_temp=");
      Serial.println(predict_temp);
      double diff_temp = predict_temp - target_temp;
      
      //if the temp will overshoot, close the valve a bit: (but close more than open, the radiator will hold the heat for ~10 more minutes)
      if (diff_temp > 0 && (diff_temp * DIFF_MULTIPLIER > HYSTERESIS_OVER) ){
        MotorClose(CLOSE_MULTIPLIER * diff_temp);
      //if the temp will be too low, open the valve a bit:
      } else if (diff_temp < 0 && (diff_temp * (-1) * DIFF_MULTIPLIER > HYSTERESIS_UNDER) ){
        //check if the drop is too sudden => opened window
        if (OpenedWindow() == true){
          window_opened = true;
          MotorClose(same_direction_threshold);
          
          //MQTT publish info about opened window
          client.publish("living_room/valve1/window_opened", "ON");
        //if window was open and is now closed
        } else if (window_opened == true){
          window_opened = false;
          //MQTT publishmqttser info window closed
          client.publish("living_room/valve1/window_opened", "OFF");
          //if temp is too low, continue heating from full blast:
          if (temps[T_SIZE-1] + 3 < target_temp){
            MotorOpen(same_direction_threshold); 
          }
        }
        //window isn't open, continue normally
        else {
          //current temp is above desired, open just a tiny bit:
          if ( (temps[T_SIZE-1] + temps[T_SIZE-2]) / 2.0 > target_temp){
            MotorOpen(OPEN_ABOVE_MULTIPLIER * diff_temp * (-1));
          //current temp is under desired, open more as usual
          } else {
            MotorOpen(OPEN_MULTIPLIER * diff_temp * (-1));
          }
        }
      }
    } else if (heating_on == false){
      MotorClose(same_direction_threshold);
    }
    time_last_react = millis();
    */
  
  }
  
  /**
   * MQTT temp sending, checks if previous block happend and if so, then it just gets recent temperature from there and doesn't get it from sensor
   * that's because the sensor could get heated up from just recent sending of temp data.
  */
  if (millis() - time_last_send > MQTT_SEND_PERIOD){
    if (gotTemp == true){
      temp = temps[T_SIZE-1];
    } else {
      temp = getTemp();
    }
    
    char tempstr[11];
    sprintf(tempstr, "%.2f", temp);
    client.publish("living_room/valve1/temp1", tempstr);
    Serial.print("MQTT publish temp1, tempstr=");
    Serial.println(tempstr);

    time_last_send = millis();
  }
}
