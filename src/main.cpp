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
// Weather and sensor emojis (8x8 pixels)
const unsigned char temp_emoji[] = {
    0b00001100,
    0b00010010,
    0b00010010,
    0b00010010,
    0b00011110,
    0b00011110,
    0b00011110,
    0b00001100
};

const unsigned char rain_emoji[] = {
    0b00111100,
    0b01111110,
    0b01111110,
    0b00111100,
    0b00001000,
    0b00010100,
    0b00001000,
    0b00000100
};

const unsigned char droplet_emoji[] = {
    0b00001000,
    0b00011100,
    0b00111110,
    0b01111111,
    0b01111111,
    0b00111110,
    0b00011100,
    0b00001000
};

const unsigned char sun_emoji[] = {
    0b00001000,
    0b00101010,
    0b00011100,
    0b01111111,
    0b00011100,
    0b00101010,
    0b00001000,
    0b00000000
};

const unsigned char moisture_emoji[] = {
    0b00001000,    // Single drop
    0b00011100,
    0b00111110,
    0b00001000,    // Second drop
    0b00011100,
    0b00111110,
    0b00001000,    // Third drop
    0b00011100
};

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
  // Serial.print(lastCommand);
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
      const float SUNLIGHT_THRESHOLD = 10; // Lux threshold for too sunny
      const bool isDry = moistP < soilThreshold;
      String weatherUpper = weather;
      weatherUpper.toUpperCase();
      const bool isRaining = weather.length() > 0 && weatherUpper.indexOf("RAIN") != -1;
      const bool isTooSunny = lux > SUNLIGHT_THRESHOLD;
      bool newCommand = (isDry && !isRaining && !isTooSunny);
      
      if (newCommand != lastCommand) {
        lastCommand = newCommand;
        String cmd = newCommand ? "TRUE" : "FALSE";
        LoRa.idle();
        const int maxRetries = 3;
        int attempt = 0;
        bool sent = false;
        // Send the command multiple times with small delays, without checking for success
        for (int i = 0; i < 3; ++i) {
          if (LoRa.beginPacket() && LoRa.print("CMD:" + cmd) && LoRa.endPacket()) {
            Serial.println("CMD:" + cmd + " sent (burst " + String(i + 1) + ")");
          } else {
            Serial.println("LoRa CMD Send Failed (burst " + String(i + 1) + ")");
          }
          delay(50); // Small delay between bursts
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
      // Set lastCommand accordingly
      if (msg == "TRUE") {
        lastCommand = true;
      } else if (msg == "FALSE") {
        lastCommand = false;
      }
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

  // Temperature reading with thermometer emoji
  oled.drawBitmap(2, 4, temp_emoji, 8, 8, SSD1306_WHITE);
  oled.setCursor(12, 5);
  oled.printf("%.1fC", tempC);

  // Humidity reading with droplet emoji
  oled.drawBitmap(64, 4, droplet_emoji, 8, 8, SSD1306_WHITE);
  oled.setCursor(74, 5);
  oled.printf("%.1f%%", humP);

  // Light level with sun emoji
  oled.drawBitmap(2, 22, sun_emoji, 8, 8, SSD1306_WHITE);
  oled.setCursor(12, 23);
  oled.printf("%.1f lx", lux);

  // Moisture reading with moisture emoji
  oled.drawBitmap(64, 22, moisture_emoji, 8, 8, SSD1306_WHITE);
  oled.setCursor(74, 23);
  oled.printf("%.0f%%", moistP);

  // Weather status with dynamic weather icon
  oled.setCursor(0, 40);
  oled.printf("Weather: %s", weather.c_str());
  
  // Draw weather icon based on conditions
  String weatherUpper = weather;
  weatherUpper.toUpperCase();
  if (weatherUpper.indexOf("RAIN") != -1) {
    oled.drawBitmap(100, 38, rain_emoji, 8, 8, SSD1306_WHITE);
  } else if (lux > 9) {
    oled.drawBitmap(100, 38, sun_emoji, 8, 8, SSD1306_WHITE);
  } else {
    // Default cloud-like pattern for other conditions
    oled.fillCircle(104, 42, 3, SSD1306_WHITE);
    oled.fillCircle(100, 42, 2, SSD1306_WHITE);
    oled.fillCircle(108, 42, 2, SSD1306_WHITE);
  }

  // Time and valve state at bottom
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    char timeStr[9];
    strftime(timeStr, sizeof(timeStr), "%H:%M:%S", &timeinfo);
    oled.setCursor(0, 54);
    oled.printf("%s", timeStr);
  }

  // Valve state
  oled.setCursor(70, 54);
  oled.printf("V:%s", valve.c_str());

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