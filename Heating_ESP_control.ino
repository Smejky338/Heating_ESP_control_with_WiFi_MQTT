/*
 * TODO:
 * termostatická kontrola teploty
 *  ^otestovat!
 * detekce otevřeného okna
 * Při neúspěšném připojení otevře AP a umožní připojit pro nastavení 
 *  ^https://github.com/esp8266/Arduino/blob/master/libraries/ESP8266WiFi/examples/WiFiScan/WiFiScan.i
 *  ^https://github.com/esp8266/Arduino/blob/master/libraries/ESP8266WiFi/examples/WiFiAccessPoint/WiFiAccessPoint.ino
*/
#include <ESP8266WiFi.h>

//#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include <NTPClient.h>

//DS18B20 reading using lib by Rob Tillaart
#include <OneWire.h>
#include <DS18B20.h>

//PID
//#include <PID_v1.h>

#include "QuickPID.h"

//MQTT:
#include <PubSubClient.h>

#include "credentials.h"

const int DSpin = D5;
OneWire oneWire(DSpin);
DS18B20 sensor(&oneWire);

//WiFi fallback credentials, should be included from credentials.h
#ifndef STASSID
  #define STASSID "WiFiSSID"
  #define STAPSK  "WiFipassword"
#endif

//MQTT fallback credentials, should be included from credentials.h
#ifndef MQTTCRED
  const char* mqttServer = "192.168.X.XXX";//"192.168.1.122";
  const int mqttPort = 1883;
  const char* mqttUser = "ESPclient";
  const char* mqttPassword = "mqttpassword";

  //password for uploading OTA updates to ESP
  const char* OTApassword = "otapassword";
#endif

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
double temps[T_SIZE] = {21,21,21,21,21,21};

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
 * using library available from https://github.com/Dlloydev/QuickPID
*/
int16_t Setpoint = target_temp, Input, Output;

float aggKp = 3, aggKi = 0.25, aggKd = 0.2;
float consKp = 1, consKi = 0.05, consKd = 0.25;
float aggPOn = 0.8; // Range is 0.0 to 1.0 (1.0 is 100% P on Error, 0% P on Measurement)
float consPOn = 0.0; // Range is 0.0 to 1.0 (0.0 is 0% P on Error, 100% P on Measurement)

/**Stores previous position of valve, used to determine motor movement
 * 0 = fully closed valve, 255 = open
*/
int8_t prev_position;

//Specify the links and initial tuning parameters
QuickPID myQuickPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, aggPOn, DIRECT);


bool window_opened = false;
#define HYSTERESIS_OPENED_WINDOW 1
#define HYSTERESIS_CLOSED_WINDOW 0.3

/*NTP section*/
//update every 48 hrs.
#define NTP_PERIOD 1000*60*60*48

#define PragueOffset 3600 //UTC+1

WiFiUDP ntpUDP;
//ptr WiFiUDP, serveraddr, offset[s], update interval [ms]
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 3600, NTP_PERIOD);

unsigned long last_ntp = 0;

bool decalc_done = false;

//period between sending current temperature
#define MQTT_SEND_PERIOD 30*1000

unsigned long time_last_send = 0;

WiFiClient espClient;
PubSubClient client(espClient);

/**
 * Initializes WiFi connection and starts endpoint for OTA updates. 
 * If WiFi connection wasn't succesful, restarts whole ESP and starts again.
 */
void WifiOtaON() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(STASSID, STAPSK);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  //set password for uploading new code to ESPs
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

//initializates MQTT connection. Subscribing happens in setup()
//inspired from https://techtutorialsx.com/2017/04/09/esp8266-connecting-to-mqtt-broker/
bool MQTTinit(){
  client.setServer(mqttServer, mqttPort);
  client.setCallback(handleMQTT);
  unsigned cnt = 0;
  char mqttID[20]="ESPvalve-";
  
  strcat(mqttID, String(random(0xffff), HEX).c_str());
  
  Serial.println("Trying to connect to MQTT...");
  
  if (client.connect(mqttID, mqttUser, mqttPassword )) {
    Serial.println("MQTT connected");  
    
    //SUB to topics valve is interested in to get info about for its operation (controls, weather info, ...)
    client.subscribe("living_room/valve1/desired_temp/set");
    client.subscribe("living_room/valve1/mode/set");
    
    //MQTT publish info window closed
    client.publish("living_room/valve1/window_opened", "OFF");
    
    return true;
  } else {
    Serial.print("failed with state ");
    Serial.println(client.state());
    return false;
  }
}

//handles incoming MQTT messages.
void handleMQTT(char* topic, byte* payload, unsigned int length){
  char *pomtext = (char*) malloc( sizeof(char) * ( length + 1 ) );

  Serial.print("\nMessage arrived in topic: ");
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

  //DBG vypis
  Serial.print("handleMQTT pomtext=");
  Serial.println(pomtext);
  
  if (!strcmp(topic, "living_room/valve1/desired_temp/set")){
    target_temp = atof(pomtext);

    Setpoint = target_temp;
    Serial.print("New target_temp=");
    Serial.println(target_temp);
  } else if (!strcmp(topic, "living_room/valve1/mode/set")){//off|heat
    Serial.print("mode set: ");
    Serial.print(strncmp(pomtext, "heat", 4));
    Serial.println(strncmp(pomtext, "off", 3));
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

double getTemp() {
  sensor.requestTemperatures();
  
  while (!sensor.isConversionComplete());  // wait until sensor is ready

  return sensor.getTempC();
}

void TempsPushBack(double newtemp){
  unsigned i = 0;
  for (; i < T_SIZE - 1 ; i++){
    temps[i] = temps[i+1];
  }
  temps[i] = newtemp;
}

/**
 * Calculates possible future temperature
 * Returns predicted future temperature based on previous 4
 * just an average of simple linear extrapolations 
 */
double PredictTemp(){
  double sum = 0;
  for(unsigned i = 0; i < T_SIZE - 1; i++){
    sum += temps[i+1] - temps[i];
  }
  return temps[T_SIZE-1] + sum / (T_SIZE-1);
}

/* Returns true if temperature drop is over hysteresis
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
 * Checks if it's time for decalc and do it
 */
bool Decalc(){
  //getDay returns 0-6 starting Sunday
  if (timeClient.getDay() == 6 && timeClient.getHours() == 11 && decalc_done == false){
    decalc_done = true;
    MotorOpen(same_direction_threshold-1);
    delay(same_direction_threshold-1);
    MotorClose(same_direction_threshold-1);
    
    return true;
  } else if (timeClient.getDay() == 6 && timeClient.getHours() > 12) {
    decalc_done = false;
  }
  return false;
}

/**
 * Opens motor and sets var time_until declaring how long should it be opening for.
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
 * return true when canceling action, false when no action canceled.
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

void setup() {
  // initialize GPIO pins:
    pinMode(T1, OUTPUT);
    pinMode(T3, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(LED_BUILTIN, LOW);

  //initialize serial communication and OTA:
  Serial.begin(115200);
  Serial.println("Booting");

  WifiOtaON();
  
  //initialize sensor and measure first temp:
  sensor.begin();
  //sensor.setResolution(12);
  temps[0] = getTemp();
  for (int i=1; i<T_SIZE; i++){
    temps[i] = temps[i-1];
  }
  

  timeClient.begin();
  timeClient.update();

  MQTTinit();

  
  Input = temps[T_SIZE-1];
  Setpoint = target_temp;
  
  //turn the PID on
  myQuickPID.SetMode(AUTOMATIC);
 
  delay(5000);//wait for the original board to do its bootup thing...

  MotorOpen(same_direction_threshold);
  prev_position = 255;

  time_last_react = 0;
}

void loop() {
  char temps_text[32];
  double temp;
  bool gotTemp = false;
  int8_t diff_position;

  //Handle possible OTA updates:
  ArduinoOTA.handle();

  CheckMotorTimeUntil();

  if (!client.connected()){
    digitalWrite(LED_BUILTIN, LOW);//LED ON
    MQTTinit();
  } else {
    digitalWrite(LED_BUILTIN, HIGH);//LED OFF
    //keep MQTT connection and get possible messages with updates
    client.loop();
  }
  
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
    /*if (abs(target_temp - Input) < 1) { //we're close to setpoint, use conservative tuning parameters
      myQuickPID.SetTunings(consKp, consKi, consKd, consPOn);
    } else {
      */
      //we're far from setpoint, use aggressive tuning parameters
      myQuickPID.SetTunings(aggKp, aggKi, aggKd, aggPOn);
    /*}*/
    
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
            MotorOpen(abs(diff_position) / 255 * same_direction_threshold);
          } else {
            MotorClose(abs(diff_position) / 255 * same_direction_threshold);
          }
          prev_position = Output;
        }
        
        char outputstr[16];//just to be sure in case int somehow becomes bigger than 16b
        sprintf(outputstr, "%d", Output);
        client.publish("living_room/valve1/PIDvalue", outputstr);
      }
      //log that reaction happened:
      time_last_react = millis();
    } else if (heating_on == false){
      MotorClose(same_direction_threshold);
    }
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

/*
  //DBG
  MotorOpen(same_direction_threshold);
  prev_position = 255;
*/

  /*MQTT temp sending, checks if previous block happend and if so, then it just gets recent temperature from there and doesn't get it from sensor
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

  //delay(200);
}
