#include <DNSServer.h>

#if defined(ESP32)
  #include <WiFi.h>
  #include <AsyncTCP.h>
#else
  #include <ADS1118.h>
  #include <ESP8266WiFi.h>
  #include <ESPAsyncTCP.h>

#endif

#include <ESPUI.h>
#include <AsyncElegantOTA.h>

const byte DNS_PORT = 53;
IPAddress apIP(192, 168, 1, 1);
DNSServer dnsServer;




const char * ssid = "Exellent";
const char * APId = "ESP-WaterControl";
const char * password = "Exellenz";
const char * hostname = "ESP-WaterControl";
uint16_t valve0;
uint16_t valve1;
uint16_t valve2;
uint16_t valve3;
uint16_t status;
uint16_t LOG;
uint16_t graphId;
uint16_t gauge1;
int pin = 0;
long timers[] = {0,0,0,0,0};
int beregnungs_stop[] = {80,80,80,80,80};
int beregnung_ab[] = {30, 30,30,30,30};
long max_beregnung[] = {1*60*1000, 2*60*1000, 3*60*1000, 4*60*1000};
int measuredVal = 0;
int soilMoisturePercent = 0;
int previousSoilMoisturePercent = 0;

#if defined(ESP32)
  const int sensorPin = 34;
  int relayPin[] = {32,33,25,26};
  int airVal = 3478;
  int waterVal = 772;
#else
  const int sensorPin = A0;
  int relayPin[] = {D0,D5,D6,D7};
  int airVal = 300;
  int waterVal = 1326;

#endif




void logfunction(String text) {
  Serial.println(text);
  ESPUI.updateControlValue(LOG, text);
}

void measureSoil() {
  measuredVal = analogRead(sensorPin);
  soilMoisturePercent = map(measuredVal, airVal, waterVal, 0, 100);
  if (soilMoisturePercent > 100) {
    logfunction("Error: Moisture too high: "+ String(measuredVal)+"! Please calibrate." );
  } else if (soilMoisturePercent < 0) {
    logfunction("Error: Moisture too low: "+ String(measuredVal)+"! Please calibrate.");
  } else if (soilMoisturePercent <= 100 &&soilMoisturePercent >= 0) {
    Serial.print("Feuchtigkeit: ");
    Serial.print(soilMoisturePercent);
    Serial.println(" %");
    ESPUI.addGraphPoint(graphId, soilMoisturePercent);
    ESPUI.updateGauge(gauge1, soilMoisturePercent, -1);
    if (soilMoisturePercent != previousSoilMoisturePercent) {
      previousSoilMoisturePercent = soilMoisturePercent;
    }

  }
  timers[0] =0;
}

void rawVals(Control * sender, int value) {

  measuredVal = analogRead(sensorPin);

  logfunction("Current value: "+ String(measuredVal));
  Serial.print("Current Air value: ");
  Serial.println(airVal);
  Serial.print("Current Water value: ");
  Serial.println(waterVal);
  

}


void measureAir(Control * sender, int type) {
  switch (type) {
    case B_DOWN:
      measuredVal = analogRead(sensorPin);
      airVal = measuredVal;
      logfunction("NEW AIR value: " + String(airVal));
      
      break;
  }
}

void measureWater(Control * sender, int type) {
  switch (type) {
    case B_DOWN:
      measuredVal = analogRead(sensorPin);
      waterVal = measuredVal;
      logfunction("NEW WATER value: " + String(waterVal));
      break;
  }
}

void clearGraph(Control * sender, int type) {
  logfunction("Graph should be clean now");
  ESPUI.clearGraph(graphId);

}

void slider1_max(Control * sender, int type) {
  
  beregnungs_stop[0] = sender->value.toInt();
  logfunction("Beregnungs Zone " +String(sender->id) + " obere grenze : "+ beregnungs_stop[0] );
}
void slider1_min(Control * sender, int type) {
  beregnung_ab[0] = sender->value.toInt();
  logfunction("Beregnungs Zone " +String(sender->id) + " untere grenze : "+ beregnung_ab[0] );
}
void slider2_max(Control * sender, int type) {
  beregnungs_stop[1] = sender->value.toInt();
  logfunction("Beregnungs Zone " +String(sender->id) + " obere grenze : "+ beregnungs_stop[1] );
}
void slider2_min(Control * sender, int type) {
  beregnung_ab[1] = sender->value.toInt();
  logfunction("Beregnungs Zone " +String(sender->id) + " untere grenze : "+ beregnung_ab[1] );
}
void slider3_max(Control * sender, int type) {
  beregnungs_stop[2] = sender->value.toInt();
  logfunction("Beregnungs Zone " +String(sender->id) + " obere grenze : "+ beregnungs_stop[2] );
}
void slider3_min(Control * sender, int type) {
  beregnung_ab[2] = sender->value.toInt();
  logfunction("Beregnungs Zone " +String(sender->id) + " untere grenze : "+ beregnung_ab[2] );
}
void slider4_max(Control * sender, int type) {
  beregnungs_stop[3] = sender->value.toInt();
  logfunction("Beregnungs Zone " +String(sender->id) + " obere grenze : "+ beregnungs_stop[3] );
}
void slider4_min(Control * sender, int type) {
  beregnung_ab[3] = sender->value.toInt();
  logfunction("Beregnungs Zone " +String(sender->id) + " untere grenze : "+ beregnung_ab[3] );
}
void max_raintime(Control * sender, int type) {
  max_beregnung[(sender->id-27)] = sender->value.toInt()*60*1000;
  logfunction("raintime Zone " +String(sender->id-27) +" gesetzt: "+ max_beregnung[(sender->id-27)] );
}


void ValveFunction(Control * sender, int type) {
  switch (type) {
    case B_DOWN:
      
      pin = relayPin[(sender->id)-16];
      if (digitalRead(pin) == HIGH){
        ESPUI.getControl(sender->id)->color =  ControlColor::Wetasphalt;
        ESPUI.updateControlValue(sender->id ,"CLOSE");
        ESPUI.updateControl(sender->id);
        digitalWrite(pin, LOW);
        

      }
      else{
      logfunction(String(sender->id)+String(sender->label)+": OPEN");
      ESPUI.getControl(sender->id)->color =  ControlColor::Carrot;
      ESPUI.updateControlValue(sender->id ,"OPEN");
      ESPUI.updateControl(sender->id);
      digitalWrite(pin, HIGH);
      timers[(sender->id)-15] =0;
      logfunction("Timer set to 0: "+String((sender->id)-15));
      }
      // logfunction("Switch Pin "+String(pin));
     break;
    
      }
 }


void ValveClose(int valvenr){
    	logfunction("Valve should close: "+String(valvenr));
      pin = relayPin[valvenr];
      ESPUI.getControl(valvenr+15)->color =  ControlColor::Wetasphalt;
      ESPUI.getControl(valvenr+15)->value = "CLOSE";
      ESPUI.updateControl(valvenr+15);
      digitalWrite(pin, LOW);
      timers[valvenr] =0;
     
}




void setup(void) {
  Serial.begin(115200);
  // SETUP Pin Modes 
  pinMode(sensorPin, INPUT);
  for (int i=0; i< 4; i++) {
  pinMode(relayPin[i], OUTPUT);
  }

  #if defined(ESP32)
  WiFi.setHostname(hostname);
  #else
    WiFi.hostname(hostname);
  #endif
  

  // try to connect to existing network
  WiFi.begin(ssid, password);
  Serial.print("\n\nTry to connect to existing network");

  {
    uint8_t timeout = 20;

    // Wait for connection, 10s timeout
    do {
      delay(500);
      Serial.print(".");
      timeout--;
    } while (timeout &&WiFi.status() != WL_CONNECTED);

    // not connected -> create hotspot
    if (WiFi.status() != WL_CONNECTED) {
      Serial.print("\n\nCreating hotspot");

      WiFi.mode(WIFI_AP);
      WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
      WiFi.softAP(APId);

      timeout = 5;

      do {
        delay(500);
        Serial.print(".");
        timeout--;
      } while (timeout);
    }
  }

  dnsServer.start(DNS_PORT, "*", apIP);

  Serial.println("\n\nWiFi parameters:");
  Serial.print("Mode: ");
  Serial.println(WiFi.getMode() == WIFI_AP ? "Station" : "Client");
  Serial.print("IP address: ");
  Serial.println(WiFi.getMode() == WIFI_AP ? WiFi.softAPIP() : WiFi.localIP());

  uint16_t tab0 = ESPUI.addControl(ControlType::Tab, "Control", "Control");
  uint16_t tab1 = ESPUI.addControl(ControlType::Tab, "LiveData", "Data");
  uint16_t tab2 = ESPUI.addControl(ControlType::Tab, "Calibration", "Calibration");
  // uint16_t tab3 = ESPUI.addControl(ControlType::Tab, "Configuration", "Setttings");
  uint16_t tab4 = ESPUI.addControl(ControlType::Tab, "Configuration Z1", "ZONE 1");
  uint16_t tab5 = ESPUI.addControl(ControlType::Tab, "Configuration Z2", "ZONE 2");
  uint16_t tab6 = ESPUI.addControl(ControlType::Tab, "Configuration Z3", "ZONE 3");
  uint16_t tab7 = ESPUI.addControl(ControlType::Tab, "Configuration Z4", "ZONE 4");

  // shown above all tabs
  status = ESPUI.addControl(ControlType::Label, "Status:", "All fine", ControlColor::Turquoise);
  LOG = ESPUI.addControl(ControlType::Label, "LOG:", "-", ControlColor::Turquoise);

 
  // tabbed controls
  graphId = ESPUI.addControl(ControlType::Graph, "Sensor1", "Humidity", ControlColor::Wetasphalt, tab1);
  gauge1 = ESPUI.addControl(ControlType::Gauge, "Sensor1", "Humidity:", ControlColor::Carrot, tab1);
  ESPUI.addControl(ControlType::Button, "Clear Graph", "Clear", ControlColor::Peterriver, tab1, &clearGraph);

  ESPUI.addControl(ControlType::Button, "Calibrate ( in Air )", "Press", ControlColor::Peterriver, tab2, &measureAir);
  ESPUI.addControl(ControlType::Button, "Calibrate ( in Water )", "Press", ControlColor::Alizarin, tab2, &measureWater);
  ESPUI.addControl(ControlType::Button, "Print Raw Value", "Press", ControlColor::Emerald, tab2, &rawVals);


  valve0 =   ESPUI.addControl(ControlType::Button, "Valve0", "Open", ControlColor::Wetasphalt, tab0, &ValveFunction);
  valve1 =   ESPUI.addControl(ControlType::Button, "Valve1", "Open", ControlColor::Wetasphalt, tab0, &ValveFunction);
  valve2 =   ESPUI.addControl(ControlType::Button, "Valve2", "Open", ControlColor::Wetasphalt, tab0, &ValveFunction);
  valve3 =   ESPUI.addControl(ControlType::Button, "Valve3", "Open", ControlColor::Wetasphalt, tab0, &ValveFunction);
 
 
 
  uint16_t zone1_min = ESPUI.addControl(ControlType::Slider, "Schwellenwert", "30", ControlColor::Alizarin, tab4, &slider1_min);
  uint16_t zone1_max = ESPUI.addControl(ControlType::Slider, "BeregnungStoppen bei", "80", ControlColor::Peterriver, tab4, &slider1_max);

  uint16_t zone2_min = ESPUI.addControl(ControlType::Slider, "Schwellenwert", "30", ControlColor::Alizarin, tab5, &slider2_min);
  uint16_t zone2_max = ESPUI.addControl(ControlType::Slider, "Beregnung Stoppen bei", "80", ControlColor::Peterriver, tab5, &slider2_max);
  
  uint16_t zone3_min = ESPUI.addControl(ControlType::Slider, "Schwellenwert", "30", ControlColor::Alizarin, tab6, &slider3_min);
  uint16_t zone3_max = ESPUI.addControl(ControlType::Slider, "Beregnung Stoppen bei", "80", ControlColor::Peterriver, tab6, &slider3_max);
  
  uint16_t zone4_min = ESPUI.addControl(ControlType::Slider, "Schwellenwert", "30", ControlColor::Alizarin, tab7, &slider4_min);
  uint16_t zone4_max = ESPUI.addControl(ControlType::Slider, "Beregnung Stoppen bei", "80", ControlColor::Peterriver, tab7, &slider4_max);

  ESPUI.addControl(ControlType::Number, "Maximale Beregnungszeit in min ZONE1:", "1", ControlColor::Alizarin, tab4, &max_raintime);
  ESPUI.addControl(ControlType::Number, "Maximale Beregnungszeit in min ZONE2:", "2", ControlColor::Alizarin, tab5, &max_raintime); 
  ESPUI.addControl(ControlType::Number, "Maximale Beregnungszeit in min ZONE3:", "3", ControlColor::Alizarin, tab6, &max_raintime);
  ESPUI.addControl(ControlType::Number, "Maximale Beregnungszeit in min ZONE4:", "4", ControlColor::Alizarin, tab7, &max_raintime);
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
  ESPUI.beginSPIFFS("DFR25\n WaterControl");
  
}

void debug(){
    for (int a=0; a< 5; a++) {
    Serial.println("Timer"+String(a)+": "+ timers[a]);
    }
    for (int b=0; b< 4; b++) {
    Serial.println("Pin"+String(b)+": "+ digitalRead(relayPin[b]));
    }
}


void  timer_function(void){
   
     
  if (millis() - timers[0] > 30000) {
    measureSoil();
    debug();
    timers[0] = millis();
  }
  // Zone 1 Beregnung Timer
  if (millis() - timers[1] > max_beregnung[0] && digitalRead(relayPin[0]) == HIGH) {
    ValveClose(1);
    timers[1] = millis();
  }
  // Zone 2 Beregnung Timer
  if (millis() - timers[2] > max_beregnung[1] && digitalRead(relayPin[1]) == HIGH) {
    ValveClose(2);
    timers[2] = millis();
  }
  // Zone 3 Beregnung Timer
  if (millis() - timers[3] > max_beregnung[2] && digitalRead(relayPin[2]) == HIGH) {
    ValveClose(3);
    timers[3] = millis();
  }
  // Zone 4 Beregnung Timer
  if (millis() - timers[4] > max_beregnung[3] && digitalRead(relayPin[3]) == HIGH) {
    ValveClose(4);
    timers[4] = millis();
  }
  
}
// TODO Start on MOISTURE AND STOP ON MOISTURE

// TODO STORE SETTINGS AS JSON AND LOAD SETTINGS ON BOOT
  void saveValues(){
  
  }
  void loadValues(){
  }

void loop(void) {
  dnsServer.processNextRequest();
  timer_function();
}
