#include <config.h>
#if defined(ESP32)
  #include <WiFi.h>
  #include <AsyncTCP.h>
#else
  #include <ESP8266WiFi.h>
  #include <ESPAsyncTCP.h>
  #include <ADS1115_WE.h>
  #include<Wire.h>
  #define I2C_ADDRESS 0x48
  #define I2C_SDA (15)
  #define I2C_SCL (2)  
    // ESP 12-F SPI Definition
  #define PIN_SPI_SS   (9)
  #define PIN_SPI_MOSI (13)
  #define PIN_SPI_MISO (10)
  #define PIN_SPI_SCK  (14)

  // static const uint8_t SS    = PIN_SPI_SS;
  // static const uint8_t MOSI  = PIN_SPI_MOSI;
  // static const uint8_t MISO  = PIN_SPI_MISO;
  // static const uint8_t SCK   = PIN_SPI_SCK;
  ADS1115_WE adc = ADS1115_WE(I2C_ADDRESS);
#endif
#include <ArduinoJson.h>
#include <DNSServer.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <ESPUI.h>
#if defined(USE_RTC)
  #include <Wire.h>
  #include <RtcDS1307.h>
#endif
#include <map>

const byte DNS_PORT = 53;
IPAddress apIP(192, 168, 179, 2);
DNSServer dnsServer;
const char* mqtt_server = "YOUR_MQTT_BROKER_IP_ADDRESS";


//WIFI
struct wifi_config {
  char ssid[32];
  char pass[64];
} wifi = {
  "YOUR_WIFI_SSID",
  "YOUR_WIFI_PASSWORD"
};

const char * APId = "ESP-WaterControl";
const char * hostname = "ESP-WaterControl";

// MQTT
#if defined(USE_MQTT)
  WiFiClient espClient;
  PubSubClient mqttClient(espClient);
  struct mqtt_config {
    char user[32];
    char pass[64];
    char client_id[32];
    char topic[32];
  } mqtt = {
    "YOUR_MQTT_USERNAME",
    "YOUR_MQTT_PASSWORD",
    "YOUR_MQTT_CLIENT_ID",
    "Watercontrol/"
  };
  long lastMsg = 0;
  char msg[50];
  char buffer [10];
#endif

// ESPUI
uint16_t valve0;
uint16_t valve1;
uint16_t valve2;
uint16_t valve3;
uint16_t status;
uint16_t LOG;
uint16_t graphId;
uint16_t gauge1;
int pin = 0;

#if defined(USE_RTC)
  // RTC
  RtcDS1307<TwoWire> Rtc(Wire);
  RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
  // Timers and Pins
  long timers[] = {0,0,0,0,0};
  int rain_stop[] = {80,80,80,80,80};
  int rain_start[] = {30, 30,30,30,30};
  long max_raintime[] = {1*60*1000, 2*60*1000, 3*60*1000, 4*60*1000};

  // Values
  int measuredVal = 0;
  int soilMoisturePercent = 50;
  int previousSoilMoisturePercent = 50;
  int selectedZone = 0;
  std::map<std::string, int> zone_assoc { {"Zone 1", 0}, {"Zone 2", 1}, {"Zone 3", 2}, {"Zone 4", 3} };
  std::map<int,uint16_t> valvesUI;
  const char* on_off[] = {"OFF", "ON"};
#endif
#if defined(ESP32)
  int sensors[] = {35,34,32,33};
  int relayPin[] = {11,8,7,6};
  int buttonPin[] = {11,10,9,14};
  int airVal = 3478;
  int waterVal = 772;
#else
  int sensors[] = {ads1118.AIN_0,ads1118.AIN_1,ads1118.AIN_2,ads1118.AIN_3};
  int relayPin[] = {D0,D5,D6,D7};
  int airVal = 300;
  int waterVal = 1326;

#endif
  



int label_to_int(const char * value)
{
 return zone_assoc[value];
}

void logfunction(const String &text ) {
  Serial.print(text);
  ESPUI.updateControlValue(LOG, text);
}

void measureSoil(int zone) {
  measuredVal = analogRead(sensors[zone]);
  soilMoisturePercent = map(measuredVal, airVal, waterVal, 0, 100);
  if (soilMoisturePercent > 100) {
    logfunction("Error Zone"+ String(zone)+":  Moisture too high: "+ String(measuredVal)+"! Please calibrate." );
  } else if (soilMoisturePercent < 0) {
    logfunction("Error Zone "+ String(zone)+": Moisture too low: "+ String(measuredVal)+"! Please calibrate.");
  } else if (soilMoisturePercent <= 100 &&soilMoisturePercent >= 0) {
    char * mqtt_topic = mqtt.topic;
    strcat(mqtt_topic,"Zone/");
    itoa(zone, buffer, 1);
    strcat(mqtt_topic,buffer);
    strcat(mqtt_topic,"/moisture");
    mqttClient.publish(mqtt_topic, String(soilMoisturePercent).c_str());
    logfunction("Feuchtigkeit Zone "+ String(zone)+": ");
    logfunction(String(soilMoisturePercent));
    logfunction("\n %");
    ESPUI.addGraphPoint(graphId, soilMoisturePercent);
    ESPUI.updateGauge(gauge1, soilMoisturePercent, -1);
    if (soilMoisturePercent != previousSoilMoisturePercent) {
      previousSoilMoisturePercent = soilMoisturePercent;
    }

  }
  // timers[0] =0;
}

void rawVals(Control * sender, int value) {

  measuredVal = analogRead(sensors[selectedZone]);

  logfunction("Current value: "+ String(measuredVal));
  logfunction("Current Air value: ");
  logfunction(String(airVal));
  logfunction("Current Water value: ");
  logfunction(String(waterVal));
    
  

}

void debug_analog(){
for (int i = 0; i < 4; i++) {
  measuredVal = analogRead(sensors[i]);

  logfunction("analog redout channel: "+ String(i) +"  val:" + String(measuredVal));
  char mqttString[8];
  dtostrf(measuredVal, 1, 2, mqttString);
  char * mqtt_topic = mqtt.topic;
  strcat(mqtt_topic,"Zone/");
  itoa(i, buffer, 1);
  strcat(mqtt_topic,buffer);
  strcat(mqtt_topic,"/raw/read");
  mqttClient.publish(mqtt_topic, mqttString);
  
  // logfunction(String(ads1118.getTemperature(),6)+" C");
  }
}

void measureAir(Control * sender, int type) {
  switch (type) {
    case B_DOWN:
      measuredVal = analogRead(sensors[selectedZone]);
      airVal = measuredVal;
      logfunction("NEW AIR value: " + String(airVal));
      char mqttString[8];
      dtostrf(airVal, 1, 2, mqttString);
      char * mqtt_topic = mqtt.topic;
      strcat(mqtt_topic,"Zone/");
      itoa(selectedZone, buffer, 1);
      strcat(mqtt_topic,buffer);
      strcat(mqtt_topic,"/air");
      mqttClient.publish(mqtt_topic, mqttString);
      break;
  }
}

void measureWater(Control * sender, int type) {
  switch (type) {
    case B_DOWN:
      measuredVal = analogRead(sensors[selectedZone]);
      waterVal = measuredVal;
      logfunction("NEW WATER value: " + String(waterVal));
      char mqttString[8];
      dtostrf(waterVal, 1, 2, mqttString);
      char * mqtt_topic = mqtt.topic;
      strcat(mqtt_topic,"Zone/");
      itoa(selectedZone, buffer, 1);
      strcat(mqtt_topic,buffer);
      strcat(mqtt_topic,"/water");
      mqttClient.publish(mqtt_topic, mqttString);
      break;
  }
}

void clearGraph(Control * sender, int type) {
  logfunction("Graph should be clean now");
  ESPUI.clearGraph(graphId);

}

void slider1_max(Control * sender, int type) {
  
  rain_stop[0] = sender->value.toInt();
  logfunction("Beregnungs Zone " +String(sender->id) + " obere grenze : "+ rain_stop[0] );
}
void slider1_min(Control * sender, int type) {
  rain_start[0] = sender->value.toInt();
  logfunction("Beregnungs Zone " +String(sender->id) + " untere grenze : "+ rain_start[0] );
}
void slider2_max(Control * sender, int type) {
  rain_stop[1] = sender->value.toInt();
  logfunction("Beregnungs Zone " +String(sender->id) + " obere grenze : "+ rain_stop[1] );
}
void slider2_min(Control * sender, int type) {
  rain_start[1] = sender->value.toInt();
  logfunction("Beregnungs Zone " +String(sender->id) + " untere grenze : "+ rain_start[1] );
}
void slider3_max(Control * sender, int type) {
  rain_stop[2] = sender->value.toInt();
  logfunction("Beregnungs Zone " +String(sender->id) + " obere grenze : "+ rain_stop[2] );
}
void slider3_min(Control * sender, int type) {
  rain_start[2] = sender->value.toInt();
  logfunction("Beregnungs Zone " +String(sender->id) + " untere grenze : "+ rain_start[2] );
}
void slider4_max(Control * sender, int type) {
  rain_stop[3] = sender->value.toInt();
  logfunction("Beregnungs Zone " +String(sender->id) + " obere grenze : "+ rain_stop[3] );
}
void slider4_min(Control * sender, int type) {
  rain_start[3] = sender->value.toInt();
  logfunction("Beregnungs Zone " +String(sender->id) + " untere grenze : "+ rain_start[3] );
}

void max_raintime_function(Control * sender, int type) {
  max_raintime[(sender->id-27)] = sender->value.toInt()*60*1000;
  logfunction("raintime Zone " +String(sender->id-32) +" gesetzt: "+ max_raintime[(sender->id-27)] );
  char mqttString[8];
  dtostrf(max_raintime[(sender->id-27)], 1, 2, mqttString);
  char * mqtt_topic = mqtt.topic;
  strcat(mqtt_topic,"Zone/");
  itoa((sender->id-27), buffer, 1);
    strcat(mqtt_topic,buffer);
    strcat(mqtt_topic,"/raintime");
    mqttClient.publish(mqtt_topic, String(soilMoisturePercent).c_str());
    
  mqttClient.publish(mqtt_topic,mqttString );

}

void ValveOpen (int valvenr){
    	pin = relayPin[valvenr];
      timers[valvenr+1] =millis();
      logfunction("Valve should open: "+String(valvenr));
      char * mqtt_topic = mqtt.topic;
      strcat(mqtt_topic,"Zone/");
      itoa(valvenr,buffer,1);
      strcat(mqtt_topic,buffer);
      strcat(mqtt_topic,"/Valve");
      ESPUI.updateSwitcher(valvesUI[valvenr],true);
      digitalWrite(pin, HIGH);
      mqttClient.publish(mqtt_topic, on_off[1]);
      }
void ValveClose (int valvenr){
    	timers[valvenr+1] =0;
     char * mqtt_topic = mqtt.topic;
      strcat(mqtt_topic,"Zone/");
      itoa(valvenr,buffer,1);
      strcat(mqtt_topic,buffer);
      strcat(mqtt_topic,"/Valve");
      logfunction("Valve should close: "+String(valvenr));
      mqttClient.publish(mqtt_topic, on_off[0]);
      pin = relayPin[valvenr];
      ESPUI.updateSwitcher(valvesUI[valvenr],false);
      digitalWrite(pin, LOW);
      
}
void Valve_callback(Control * sender, int type) {
  int valveNr = label_to_int(sender->label);
   switch (type)
    {
    case S_ACTIVE:
        ValveOpen(valveNr);
        break;
    case S_INACTIVE:
        ValveClose(valveNr);
        break;
    }
 }
void ValveToggle(int valvenr){
    	pin = relayPin[valvenr];
      if (digitalRead(pin) == HIGH) {
      ValveClose(valvenr);
      } else {
      ValveOpen(valvenr);
      }
}

void selectZone(Control* sender, int value)

// TODO: Switch Sliders and Zone Config by Tab
{
   logfunction("Select: ID: "+ (sender->id));
    logfunction("Value: " + (sender->value));
    selectedZone = label_to_int(sender->value.c_str())-1;
}
void tabcallback(Control* sender, int value)
{
  auto active_tab = value;
  logfunction("Tab: " + active_tab);
}

void debug(){
    for (int a=0; a<= 4; a++) {
    logfunction("Timer"+String(a)+": "+ String(timers[a]));
    }
    for (int b=0; b<= 3; b++) {
    logfunction("Pin"+String(b)+": "+ String(digitalRead(relayPin[b])));
    }
}

void timer_checkFunction(int zone){
  if ((millis() - timers[zone+1] > max_raintime[zone]) && (digitalRead(relayPin[zone]) == HIGH)) {
  ValveClose(zone);
  timers[zone+1] = 0;
  }

}

void  timer_function(void){
  
  if (millis() - timers[0] > 15000) {
  for (size_t i = 0; i <= 3; i++)
  {
       measureSoil(i);
      
      
  }
  debug();
  timers[0] = millis();
  }
  for (size_t x = 0; x <= 3; x++)
  {
    timer_checkFunction(x);
  }
  
  
}

void IRAM_ATTR button_press(){
  for (int i=0; i< 4; i++) {
    if (digitalRead(buttonPin[i]) == LOW) {
      ValveToggle(i);
    }
  }
  

}

void mqtt_callback(char* topic, byte* message, unsigned int length) {
  // logfunction("Message arrived on topic: ");
  // logfunction(topic);
  // logfunction(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    // logfunction((char)message[i]);
    messageTemp += (char)message[i];
  }
  

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  if (String(topic) == "Watercontrol/Zone/0/Valve") {
    if (String(messageTemp) == "ON") {
      ValveOpen(0);
    }
    else if (String(messageTemp) == "OFF") {
      ValveClose(0);
    }
  }
  if (String(topic) == "Watercontrol/Zone/1/Valve") {
    if (String(messageTemp) == "ON") {
      ValveOpen(1);
    }
    else if (String(messageTemp) == "OFF") {
      ValveClose(1);
    }
  }
  if (String(topic) == "Watercontrol/Zone/2/Valve") {
    if (String(messageTemp) == "ON") {
      ValveOpen(2);
    }
    else if (String(messageTemp) == "OFF") {
      ValveClose(2);
    }
  }
  if (String(topic) == "Watercontrol/Zone/3/Valve") {
    if (String(messageTemp) == "ON") {
      ValveOpen(3);
    }
    else if (String(messageTemp) == "OFF") {
      ValveClose(3);
    }
  }

  
}
boolean mqtt_reconnect() {
  // Loop until we're reconnected
    if (mqttClient.connect(mqtt.client_id, mqtt.user, mqtt.pass)) {
      logfunction("\nconnected");
      // Subscribe
      mqttClient.subscribe(mqtt.topic);
      mqttClient.publish("Watercontrol/Status", "Connected");
    
  }
  return mqttClient.connected();
}
void mqtt_setup() {
  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setCallback(mqtt_callback);
}


     
// TODO: Start on MOISTURE AND STOP ON MOISTURE

// TODO: STORE SETTINGS AS JSON AND LOAD SETTINGS ON BOOT
// void saveValues(){
// serializeJson(json_data, json_buffer);

// }
  
void loadValues(){
 
  if (!LittleFS.begin())
  {
    Serial.println("Failed to mount LittleFS");
  }
  else
  {
    File file = LittleFS.open("/options.json", FILE_READ);

    if (!file)
    {
      Serial.println("There was an error opening the config file");
      return;
    }

    else
    {
      Serial.println("File opened!");

      StaticJsonDocument<1024> json_data;

      DeserializationError error = deserializeJson(json_data, file);
      if (error)
      {
        Serial.println("error...");
      }
      else
      {
        Serial.println("Another winner!");
        strcpy(wifi.ssid, json_data["WIFIssid"]);
        strcpy(wifi.pass, json_data["WIFIpass"]);
        strcpy(mqtt.user, json_data["MQTTUser"]);
        strcpy(mqtt.pass, json_data["MQTTPass"]);
      }
      Serial.println("");
    }

    file.close();
  }
}

void setup_theUI(){
  uint16_t tab0 = ESPUI.addControl(ControlType::Tab, "Control", "Control", None, None, &tabcallback);
  uint16_t tab1 = ESPUI.addControl(ControlType::Tab, "LiveData", "Data", None, None, &tabcallback);
  uint16_t tab2 = ESPUI.addControl(ControlType::Tab, "Calibration", "Calibration", None, None, &tabcallback);
  // uint16_t tab3 = ESPUI.addControl(ControlType::Tab, "Configuration", "Setttings");
  uint16_t tab4 = ESPUI.addControl(ControlType::Tab, "Configuration Z1", "ZONE 1", None, None, &tabcallback);
  uint16_t tab5 = ESPUI.addControl(ControlType::Tab, "Configuration Z2", "ZONE 2", None, None, &tabcallback);
  uint16_t tab6 = ESPUI.addControl(ControlType::Tab, "Configuration Z3", "ZONE 3", None, None, &tabcallback);
  uint16_t tab7 = ESPUI.addControl(ControlType::Tab, "Configuration Z4", "ZONE 4", None, None, &tabcallback);
  // auto select_cal = ESPUI.addControl( ControlType::Select, "Zone to Calibrate", "Zone1", ControlColor::Alizarin, tab2, &selectZone);
  // shown above all tabs
  status = ESPUI.addControl(ControlType::Label, "Status:", "All fine", ControlColor::Turquoise);
  LOG = ESPUI.addControl(ControlType::Label, "LOG:", "-", ControlColor::Turquoise);

  // tab 0
  // static String Valves[] {"Zone 1", "Zone 2", "Zone 3", "Zone 4"};
  // for(auto const& v : Valves) {
  //   ESPUI.addControl(ControlType::Switcher, v.c_str(), "Open", ControlColor::Wetasphalt, tab0, &ValveFunction);
  // }
  valvesUI[0] =   ESPUI.addControl(ControlType::Switcher, "Zone 1", "0", ControlColor::Wetasphalt, tab0, &Valve_callback);
  valvesUI[1] =   ESPUI.addControl(ControlType::Switcher, "Zone 2", "0", ControlColor::Wetasphalt, tab0, &Valve_callback);
  valvesUI[2] =   ESPUI.addControl(ControlType::Switcher, "Zone 3", "0", ControlColor::Wetasphalt, tab0, &Valve_callback);
  valvesUI[3] =   ESPUI.addControl(ControlType::Switcher, "Zone 4", "0", ControlColor::Wetasphalt, tab0, &Valve_callback);

  // tab 1
  graphId = ESPUI.addControl(ControlType::Graph, "Sensor1", "Humidity", ControlColor::Wetasphalt, tab1);
  gauge1 = ESPUI.addControl(ControlType::Gauge, "Sensor1", "Humidity:", ControlColor::Carrot, tab1);
  ESPUI.addControl(ControlType::Button, "Clear Graph", "Clear", ControlColor::Peterriver, tab1, &clearGraph);

  
  // tab 2
  static String optionValues[] {"Zone 1", "Zone 2", "Zone 3", "Zone 4"};
  auto zoneSelect = ESPUI.addControl(Select, "Zone Selector", "Z-Selector", Wetasphalt, tab2, selectZone);
	for(auto const& v : optionValues) {
  	ESPUI.addControl(Option, v.c_str(),v, None, zoneSelect);
  }

  ESPUI.addControl(ControlType::Button, "Calibrate ( in Air )", "Press", ControlColor::Peterriver, tab2, &measureAir);
  ESPUI.addControl(ControlType::Button, "Calibrate ( in Water )", "Press", ControlColor::Alizarin, tab2, &measureWater);
  ESPUI.addControl(ControlType::Button, "Print Raw Value", "Press", ControlColor::Emerald, tab2, &rawVals);


 
 
 
  ESPUI.addControl(ControlType::Slider, "Beregnung starten bei %", "30", ControlColor::Alizarin, tab4, &slider1_min);
  ESPUI.addControl(ControlType::Slider, "Beregnung Stoppen bei %", "80", ControlColor::Peterriver, tab4, &slider1_max);

  ESPUI.addControl(ControlType::Slider, "Beregnung starten bei %", "30", ControlColor::Alizarin, tab5, &slider2_min);
  ESPUI.addControl(ControlType::Slider, "Beregnung Stoppen bei %", "80", ControlColor::Peterriver, tab5, &slider2_max);
  
  ESPUI.addControl(ControlType::Slider, "Beregnung starten bei %", "30", ControlColor::Alizarin, tab6, &slider3_min);
  ESPUI.addControl(ControlType::Slider, "Beregnung Stoppen bei %", "80", ControlColor::Peterriver, tab6, &slider3_max);
  
  ESPUI.addControl(ControlType::Slider, "Beregnung starten bei %", "30", ControlColor::Alizarin, tab7, &slider4_min);
  ESPUI.addControl(ControlType::Slider, "Beregnung Stoppen bei %", "80", ControlColor::Peterriver, tab7, &slider4_max);


  ESPUI.addControl(ControlType::Number, "Maximale Beregnungszeit in min ZONE1:", "1", ControlColor::Alizarin, tab4, &max_raintime_function);
  ESPUI.addControl(ControlType::Number, "Maximale Beregnungszeit in min ZONE2:", "2", ControlColor::Alizarin, tab5, &max_raintime_function); 
  ESPUI.addControl(ControlType::Number, "Maximale Beregnungszeit in min ZONE3:", "3", ControlColor::Alizarin, tab6, &max_raintime_function);
  ESPUI.addControl(ControlType::Number, "Maximale Beregnungszeit in min ZONE4:", "4", ControlColor::Alizarin, tab7, &max_raintime_function);
}

void setup(void) {
  
  Serial.begin(115200);
  
  // SETUP Pin Modes 
  
  
  // SETUP RTC 
  
  for (int i=0; i< 4; i++) {
  pinMode(relayPin[i], OUTPUT);
  }
  #if defined(ESP32)
    for (int i=0; i< 4; i++) {
    pinMode(sensors[i], INPUT_PULLUP);
    pinMode(buttonPin[i], INPUT_PULLUP);
    attachInterrupt(buttonPin[i],button_press, FALLING);
    }
  #endif
  // SETUP RTC 
  #if defined(USE_RTC)
    Wire.begin(21,22);
    Rtc.Begin();
    RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__); 
  #endif
  #if defined(ESP32)
  WiFi.setHostname(hostname);
  #else
    WiFi.hostname(hostname);
    Wire.begin(I2C_SDA,I2C_SCL);
    if(!adc.init()){
    Serial.println("ADS1115 not connected!");
    adc.setVoltageRange_mV(ADS1115_RANGE_6144);
  #endif
  

  // try to connect to existing network
  WiFi.begin(wifi.ssid, wifi.pass);
  logfunction("\n\nTry to connect to existing network");

  {
    uint8_t timeout = 20;

    // Wait for connection, 10s timeout
    do {
      delay(500);
      logfunction(".");
      timeout--;
    } while (timeout &&WiFi.status() != WL_CONNECTED);

    // not connected -> create hotspot
    if (WiFi.status() != WL_CONNECTED) {
      logfunction("\n\nCreating hotspot");

      WiFi.mode(WIFI_AP);
      WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
      WiFi.softAP(APId);

      timeout = 5;

      do {
        delay(500);
        logfunction(".");
        timeout--;
      } while (timeout);
    }
  }

  // JSON init
  StaticJsonDocument<200> json_data;

  dnsServer.start(DNS_PORT, "*", apIP);
  Serial.print("\n\n\nWiFi parameters:");
  Serial.print("Mode: ");
  Serial.print(WiFi.getMode() == WIFI_AP ? "Station" : "Client");
  Serial.print("IP address: ");
  Serial.print(WiFi.getMode() == WIFI_AP ? WiFi.softAPIP() : WiFi.localIP());
  mqtt_setup();

  // Start ADC config 
  // ads1118.begin();
  /* Changing the sampling rate. RATE_8SPS, RATE_16SPS, RATE_32SPS, RATE_64SPS, RATE_128SPS, RATE_250SPS, RATE_475SPS, RATE_860SPS*/
    // ads1118.setSamplingRate(ads1118.RATE_8SPS);

    /* Changing the input selected. Differential inputs: DIFF_0_1, DIFF_0_3, DIFF_1_3, DIFF_2_3. Single ended input: AIN_0, AIN_1, AIN_2, AIN_3*/
    // ads1118.setInputSelected(ads1118.AIN_0);

  setup_theUI();
  /*
    * .begin loads and serves all files from PROGMEM directly.
    * If you want to serve the files from SPIFFS use ESPUI.beginSPIFFS
    * (.prepareFileSystem has to be run in an empty sketch before)
    */

  // Enable this option if you want sliders to be continuous (update during move) and not discrete (update on stop)
  // ESPUI.sliderContinuous = true;

  /*
    * Optionally you can use HTTP BasicAuth. Keep in mind that this is NOT a
    * SECURE way of limiting access.
    * Anyone who is able to sniff traffic will be able to intercept your password
    * since it is transmitted in cleartext. Just add a string as username and
    * password, for example begin("ESPUI Control", "username", "password")
    */
  ESPUI.jsonInitialDocumentSize = 10000;
  ESPUI.setVerbosity(Verbosity::Verbose);
  ESPUI.begin("DFR25\n WaterControl");
  ArduinoOTA.begin();
 
}

void loop(void) {
  ArduinoOTA.handle();
  dnsServer.processNextRequest();
  timer_function();
  mqttClient.loop();
  long now = millis();
  if (now - lastMsg > 5000) {
    lastMsg = now;
  }
  // debug_analog();
}
