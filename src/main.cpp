#include <DNSServer.h>

#include <ESPUI.h>

const byte DNS_PORT = 53;
IPAddress apIP(192, 168, 1, 1);
DNSServer dnsServer;

#if defined(ESP32)
  #include <WiFi.h>
#else
  #include <ESP8266WiFi.h>

#endif

const char * ssid = "SWN.NET-1O5ZS3T2";
const char * APId = "ESP-WaterControl";
const char * password = "32133425607603846915";
const char * hostname = "ESP-WaterControl";
String Sbuffer = "----------------";
uint16_t button1;
uint16_t button2;
uint16_t button3;
uint16_t switchOne;
uint16_t status;
uint16_t LOG;
int graphId;
int gauge1;
int airVal = 3478;
int waterVal = 1326;
const int sensorPin = 34;
int measuredVal = 0;

int soilMoisturePercent = 0;
int previousSoilMoisturePercent = 0;



void logfunction(String text) {
  Serial.println(text);
  ESPUI.updateControlValue(LOG, text);
}

void measureSoil() {
  measuredVal = analogRead(sensorPin);
  soilMoisturePercent = map(measuredVal, airVal, waterVal, 0, 100);
  if (soilMoisturePercent > 100) {
    logfunction("Error: Moisture too high! Please calibrate.");
  } else if (soilMoisturePercent < 0) {
    logfunction("Error: Moisture too low! Please calibrate.");
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
}

void rawVals(Control * sender, int value) {

  measuredVal = analogRead(sensorPin);

  Serial.print("Current value: ");
  Serial.println(waterVal);

  Serial.print("Current Air value: ");
  Serial.println(airVal);
  Serial.print("Current Water value: ");
  Serial.println(waterVal);

  delay(250);

}


void measureAir(Control * sender, int type) {
    switch (type) {
      case B_DOWN:
        measuredVal = analogRead(sensorPin);
        airVal = measuredVal;
        logfunction("NEW AIR value: " + String(airVal));
        delay(250);
        break;
    }
}

void measureWater(Control * sender, int type) {
    switch (type) {
      case B_DOWN:
        measuredVal = analogRead(sensorPin);
        waterVal = measuredVal;
        logfunction("NEW WATER value: " + String(waterVal));
        delay(250);
        break;
    }
}

void clearGraph(Control * sender, int type) {
  ESPUI.clearGraph(graphId);

}

void numberCall(Control * sender, int value) {
  Serial.println(sender->value);
}

void textCall(Control * sender, int type) {
  Serial.print("Text: ID: ");
  Serial.print(sender->id);
  Serial.print(", Value: ");
  Serial.println(sender->value);
}

void slider(Control * sender, int type) {
  Serial.print("Slider: ID: ");
  Serial.print(sender->id);
  Serial.print(", Value: ");
  Serial.println(sender->value);
}

void buttonCallback(Control * sender, int type) {
  switch (type) {
    case B_DOWN:
      Serial.println("Button DOWN");
      break;

    case B_UP:
      Serial.println("Button UP");
      break;
  }
}

void buttonExample(Control * sender, int type) {
  switch (type) {
    case B_DOWN:
      Serial.println("Status: Start");
      ESPUI.updateControlValue(status, "Start");

      ESPUI.getControl(button1)->color = ControlColor::Carrot;
      ESPUI.updateControl(button1);
      break;

    case B_UP:
      Serial.println("Status: Stop");
      ESPUI.updateControlValue(status, "Stop");

      ESPUI.getControl(button1)->color = ControlColor::Peterriver;
      ESPUI.updateControl(button1);
      break;
  }
}

void selectExample(Control * sender, int value) {
  Serial.print("Select: ID: ");
  Serial.print(sender->id);
  Serial.print(", Value: ");
  Serial.println(sender->value);
}

void otherSwitchExample(Control * sender, int value) {
  switch (value) {
    case S_ACTIVE:
      Serial.print("Active:");
      break;

    case S_INACTIVE:
      Serial.print("Inactive");
      break;
  }

    Serial.print(" ");
    Serial.println(sender->id);
  }

void setup(void) {
  Serial.begin(115200);
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
  uint16_t tab3 = ESPUI.addControl(ControlType::Tab, "Configuration", "Setttings");

  // shown above all tabs
  status = ESPUI.addControl(ControlType::Label, "Status:", "All fine", ControlColor::Turquoise);
  LOG = ESPUI.addControl(ControlType::Label, "LOG:", "-", ControlColor::Turquoise);

  // uint16_t select1 = ESPUI.addControl( ControlType::Select, "Select:", "", ControlColor::Alizarin, tab1, &selectExample );
  // ESPUI.addControl( ControlType::Option, "Option1", "Opt1", ControlColor::Alizarin, select1 );
  // ESPUI.addControl( ControlType::Option, "Option2", "Opt2", ControlColor::Alizarin, select1 );
  // ESPUI.addControl( ControlType::Option, "Option3", "Opt3", ControlColor::Alizarin, select1 );

  // ESPUI.addControl( ControlType::Text, "Text Test:", "a Text Field", ControlColor::Alizarin, tab1, &textCall );

  // tabbed controls
  graphId = ESPUI.addControl(ControlType::Graph, "Sensor1", "Humidity", ControlColor::Wetasphalt, tab1);
  gauge1 = ESPUI.addControl(ControlType::Gauge, "Sensor1", "Humidity:", ControlColor::Carrot, tab1);
  ESPUI.addControl(ControlType::Button, "Clear Graph", "Clear", ControlColor::Peterriver, tab1, &clearGraph);

  button1 = ESPUI.addControl(ControlType::Button, "Calibrate ( in Air )", "Press", ControlColor::Peterriver, tab2, &measureAir);
  button2 = ESPUI.addControl(ControlType::Button, "Calibrate ( in Water )", "Press", ControlColor::Alizarin, tab2, &measureWater);
  button3 = ESPUI.addControl(ControlType::Button, "Print Raw Value", "Press", ControlColor::Emerald, tab2, &rawVals);


  ESPUI.addControl(ControlType::Button, "Valve1", "Open", ControlColor::Wetasphalt, tab0, &buttonExample);
  ESPUI.addControl(ControlType::Button, "Valve2", "Open", ControlColor::Wetasphalt, tab0, &buttonExample);
  ESPUI.addControl(ControlType::Button, "Valve2", "Open", ControlColor::Wetasphalt, tab0, &buttonExample);
  // ESPUI.addControl( ControlType::PadWithCenter, "Pad with center", "", ControlColor::Sunflower, tab2, &padExample );
  // ESPUI.addControl( ControlType::Pad, "Pad without center", "", ControlColor::Carrot, tab3, &padExample );
  // switchOne = ESPUI.addControl( ControlType::Switcher, "Switch one", "", ControlColor::Alizarin, tab3, &switchExample );
  // ESPUI.addControl( ControlType::Switcher, "Switch two", "", ControlColor::None, tab3, &otherSwitchExample );
  ESPUI.addControl(ControlType::Slider, "Slider one", "30", ControlColor::Alizarin, tab3, &slider);
  ESPUI.addControl(ControlType::Slider, "Slider two", "100", ControlColor::Alizarin, tab3, &slider);
  ESPUI.addControl(ControlType::Number, "Number:", "50", ControlColor::Alizarin, tab3, &numberCall);

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


  ESPUI.begin("DRF WaterControl");
}

void loop(void) {
  dnsServer.processNextRequest();

  static long oldTime = 0;

  if (millis() - oldTime > 5000) {
    measureSoil();
    oldTime = millis();
  }
}