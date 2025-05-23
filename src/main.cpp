/*****************************************************************
 *  ESP32  •  LoRa 2-way   •  Wi-Fi + MQTT + OLED + NTP Timestamp
 *  Publishes sensor JSON to  IoT-G9
 *  Listens for commands   on  IoT-G9/cmd   (ON / OFF)
 *  Relays those commands over LoRa to the Arduino valve node
 *****************************************************************/

#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <time.h>
#include <sys/time.h>

/* ---------- Wi-Fi & MQTT ---------- */
const char* WIFI_SSID  = "Pamith";
const char* WIFI_PASS  = "11111111";
const char* MQTT_HOST  = "test.mosquitto.org";
const uint16_t MQTT_PORT = 1883;
const char* PUB_TOPIC  = "IoT-G9";
const char* VAL_TOPIC = "IoT-G9/valve";
const char* CMD_TOPIC  = "IoT-G9/cmd";
const char* SOIL_TOPIC = "IoT-G9/soil";
const char* MODE_TOPIC = "IoT-G9/mode";  // New topic for mode control
bool isManualMode = false;  // false = auto mode, true = manual mode

/* ---------- NTP ---------- */
const char* NTP_SERVER = "pool.ntp.org";
const long  GMT_OFFSET_SEC = 5 * 3600 + 30 * 60;  // Sri Lanka GMT+5:30
const int   DAYLIGHT_OFFSET_SEC = 0;
bool lastCommand = false; // Default to false
/* ---------- LoRa ---------- */
constexpr long RF_FREQ = 433E6;
constexpr uint8_t SYNC_WORD = 0xA5;
constexpr gpio_num_t L_CS = GPIO_NUM_5;
constexpr gpio_num_t L_RST = GPIO_NUM_14;
constexpr gpio_num_t L_DIO0 = GPIO_NUM_26;

/* ---------- OLED ---------- */
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
constexpr uint32_t OLED_INTERVAL = 250;

/* ---------- LED ---------- */
constexpr gpio_num_t LED_PIN = GPIO_NUM_2;

/* ---------- Objects ---------- */
WiFiClient      net;
PubSubClient    mqtt(net);

/* ---------- Shared data ---------- */
volatile bool packetReady = false;
String weather="N/A", valve="N/A"; float tempC=NAN, humP=NAN, lux=NAN, moistP=NAN;

/* ---------- Prototypes ---------- */
void connectWiFi(),  connectMQTT();
void publishJSON(),  drawOLED();
String extractStr(const String&,const String&);
float  extractFloat(const String&,const String&);
String getTimestamp();
char weatherIconAscii(const String&, float);
static float soilThreshold = 30.0;
/* ---------- LoRa ISR ---------- */
void IRAM_ATTR onPacketISR(int) { packetReady = true; }
void mqttCallback(char* topic, byte* payload, unsigned int len);

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);  digitalWrite(LED_PIN, LOW);
  Wire.begin(); Wire.setClock(400000);
  oled.begin(SSD1306_SWITCHCAPVCC,0x3C);
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(2); oled.clearDisplay(); oled.println("Awlak nane"); oled.display();

  connectWiFi();
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) Serial.println("NTP Time Sync Failed");
  else Serial.println(&timeinfo, "NTP Time: %Y-%m-%d %H:%M:%S");

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);

  SPI.begin();
  LoRa.setPins(L_CS,L_RST,L_DIO0);
  if(!LoRa.begin(RF_FREQ)){Serial.println("LoRa fail");while(true);}
  LoRa.setSyncWord(SYNC_WORD); LoRa.enableCrc();
  LoRa.onReceive(onPacketISR); LoRa.receive();
  Serial.println("Gateway ready");
}

void loop() {
  if(WiFi.status()!=WL_CONNECTED) connectWiFi();
  if(!mqtt.connected())           connectMQTT();
  mqtt.loop();

  if(packetReady){
    packetReady=false;
    String raw;
    while(LoRa.available()){
      char c=char(LoRa.read());
      if(c>=32 && c<=126) raw+=c;
    }
    weather=extractStr(raw,"Weather:");
    tempC  =extractFloat(raw,"Temp:");
    humP   =extractFloat(raw,"Hum:"); if(isnan(humP)) humP=extractFloat(raw,"Hm:");
    lux    =extractFloat(raw,"Light level:"); if(isnan(lux)) lux=extractFloat(raw,"Lux:"); if(isnan(lux)) lux=extractFloat(raw,"Lx:");
    moistP =extractFloat(raw,"Moisture:");
    valve  = extractStr(raw, "Valve:");
    
    // Publish sensor data regardless of mode
    mqtt.publish(VAL_TOPIC, valve.c_str());
    Serial.printf("RX: %s | T %.1f | H %.1f | L %.1f | M %.0f%% | V %s\n",
                  weather.c_str(), tempC, humP, lux, moistP, valve.c_str());
    publishJSON();
    drawOLED();

    // Auto mode valve control logic
    if (!isManualMode) {
      const float SUNLIGHT_THRESHOLD = 9;
      const bool isDry = moistP < soilThreshold;
      const bool isRaining = weather.indexOf("Rain") != -1;
      const bool isTooSunny = lux > SUNLIGHT_THRESHOLD;
      bool newCommand = (isDry && !isRaining && !isTooSunny);
      
      if (newCommand != lastCommand) {
        lastCommand = newCommand;
        String cmd = newCommand ? "TRUE" : "FALSE";
        LoRa.idle();
        if (LoRa.beginPacket() && LoRa.print(cmd) && LoRa.endPacket()) {
          Serial.println("Auto CMD Sent: " + cmd);
        } else {
          Serial.println("LoRa CMD Send Failed");
        }
        LoRa.receive();
      }
    }
    LoRa.receive();
  }
}

void connectWiFi(){
  Serial.print("Wi-Fi: "); WiFi.begin(WIFI_SSID, WIFI_PASS);
  while(WiFi.status()!=WL_CONNECTED){Serial.print('.');delay(300);}
  Serial.println(" connected  IP="+WiFi.localIP().toString());
  digitalWrite(LED_PIN,HIGH);
}

void connectMQTT(){
  while(!mqtt.connected()){
    Serial.print("MQTT… ");
    if(mqtt.connect("ESP32-LoRa-GW")){
      Serial.println("connected");
      mqtt.subscribe(CMD_TOPIC);
      mqtt.subscribe(SOIL_TOPIC);
      mqtt.subscribe(MODE_TOPIC);  // Subscribe to mode control topic
    }else{
      Serial.print(mqtt.state()); Serial.println(" retry"); delay(2000);
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned len) {
  String msg;
  for (uint32_t i = 0; i < len; i++) msg += (char)payload[i];
  msg.trim(); msg.toUpperCase();
  
  // Handle mode control messages
  if (strcmp(topic, MODE_TOPIC) == 0) {
    if (msg == "FALSE") {
      isManualMode = true;
      Serial.println("Switching to manual mode");
    } else if (msg == "TRUE") {
      isManualMode = false;
      Serial.println("Switching to auto mode");
    }
    return;
  }
  
  // Handle soil threshold messages
  if (strcmp(topic, SOIL_TOPIC) == 0) {
    bool isInt = true;
    for (uint32_t i = 0; i < msg.length(); ++i) {
      if (!isDigit(msg[i])) { isInt = false; break; }
    }
    if (isInt && msg.length() > 0) {
      soilThreshold = msg.toFloat();
      Serial.printf("Soil threshold set to %.1f\n", soilThreshold);
      return;
    }
  }

  // Handle valve control commands in manual mode
  if (strcmp(topic, CMD_TOPIC) == 0 && isManualMode) {
    if (msg == "TRUE" || msg == "FALSE") {
      Serial.println("Manual CMD from MQTT: " + msg);
      LoRa.idle();
      if (LoRa.beginPacket() && LoRa.print("CMD:" + msg) && LoRa.endPacket()) {
        Serial.println("Manual CMD sent: " + msg);
      } else {
        Serial.println("LoRa CMD send failed");
      }
      LoRa.receive();
    }
  }
}

void publishJSON(){
  StaticJsonDocument<256> doc;
  doc["weather"] = weather;
  doc["temp"] = tempC;
  doc["hum"] = humP;
  doc["light"] = lux;
  doc["moist"] = moistP;
  doc["timestamp"] = getTimestamp();
  char buf[256]; size_t n = serializeJson(doc, buf);
  mqtt.publish(PUB_TOPIC, buf, n);
}

void drawOLED() {
  static uint32_t last = 0;
  if (millis() - last < OLED_INTERVAL) return;
  last = millis();

  oled.clearDisplay();

  // --- Icons and Readings ---
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);

  // Thermometer icon (Temperature)
  oled.drawLine(2, 4, 2, 14, SSD1306_WHITE);           // Stem
  oled.fillCircle(2, 16, 2, SSD1306_WHITE);            // Bulb
  oled.setCursor(10, 5);
  oled.printf("T:%.1fC", tempC);

  // Humidity icon (drop)
  oled.fillTriangle(64, 4, 60, 12, 68, 12, SSD1306_WHITE); // Water drop
  oled.setCursor(72, 5);
  oled.printf("H:%.1f%%", humP);

  // Light icon (Sun)
  oled.drawCircle(2, 30, 3, SSD1306_WHITE);            // Sun core
  for (int a = 0; a < 360; a += 45) {
    int x0 = 2 + 5 * cos(a * DEG_TO_RAD);
    int y0 = 30 + 5 * sin(a * DEG_TO_RAD);
    int x1 = 2 + 7 * cos(a * DEG_TO_RAD);
    int y1 = 30 + 7 * sin(a * DEG_TO_RAD);
    oled.drawLine(x0, y0, x1, y1, SSD1306_WHITE);
  }
  oled.setCursor(10, 25);
  oled.printf("L:%.1f lx", lux);

  // Moisture icon (plant/soil)
  oled.fillCircle(66, 30, 2, SSD1306_WHITE);           // Soil dot
  oled.drawLine(66, 30, 66, 34, SSD1306_WHITE);        // Stem
  oled.drawLine(66, 32, 63, 29, SSD1306_WHITE);        // Left leaf
  oled.drawLine(66, 32, 69, 29, SSD1306_WHITE);        // Right leaf
  oled.setCursor(72, 25);
  oled.printf("M:%.0f%%", moistP);

  // Weather icon (simple cloud or rain)
  char icon = weatherIconAscii(weather, lux);
  oled.setCursor(0, 43);
  oled.printf("W: %s", weather.c_str());

  if (icon == 'R') {
    oled.fillRect(100, 42, 3, 3, SSD1306_WHITE);       // Rain drop
    oled.fillCircle(105, 40, 3, SSD1306_WHITE);        // Cloud
  } else if (icon == 'S') {
    oled.drawCircle(105, 40, 3, SSD1306_WHITE);        // Sun
  } else {
    oled.fillCircle(105, 40, 3, SSD1306_WHITE);        // Cloud
  }

  // Timestamp
  oled.setCursor(0, 54);
  oled.setTextSize(1);
  oled.printf("%s", getTimestamp().c_str());

  // Valve state
  oled.setCursor(90, 54);
  oled.printf("Valve:%s", valve.c_str());

  oled.display();
}

char weatherIconAscii(const String& w,float luxVal){
  String u=w; u.toUpperCase();
  if (u.indexOf("RAIN")!=-1) return 'R';        // Rain
  if (luxVal>9)              return 'S';        // Sunny / bright
  return 'C';                                   // Cloud/other
}

String extractStr(const String& s,const String& tag){
  int i=s.indexOf(tag); if(i==-1) return "";
  i+=tag.length(); while(i<s.length()&&s[i]==' ') ++i;
  int j=s.indexOf('|',i); if(j==-1) j=s.indexOf(',',i); if(j==-1) j=s.length();
  return s.substring(i,j);
}

float extractFloat(const String& s,const String& tag){ return extractStr(s,tag).toFloat(); }

String getTimestamp() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return "NTP_ERR";
  char buffer[20];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
  return String(buffer);
}

void (*_dummy)(char*,byte*,unsigned)=mqttCallback;