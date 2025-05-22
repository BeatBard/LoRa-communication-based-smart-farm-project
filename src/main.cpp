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
const char* CMD_TOPIC  = "IoT-G9/cmd";
const char* SOIL_TOPIC = "IoT-G9/soil";

/* ---------- NTP ---------- */
const char* NTP_SERVER = "pool.ntp.org";
const long  GMT_OFFSET_SEC = 5 * 3600 + 30 * 60;  // Sri Lanka GMT+5:30
const int   DAYLIGHT_OFFSET_SEC = 0;

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
String weather="N/A"; float tempC=NAN, humP=NAN, lux=NAN, moistP=NAN;

/* ---------- Prototypes ---------- */
void connectWiFi(),  connectMQTT();
void publishJSON(),  drawOLED();
String extractStr(const String&,const String&);
float  extractFloat(const String&,const String&);
String getTimestamp();
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

    Serial.printf("RX: %s | T %.1f | H %.1f | L %.1f | M %.0f%%\n",
                  weather.c_str(),tempC,humP,lux,moistP);
    publishJSON();
    drawOLED();

    const float SUNLIGHT_THRESHOLD = 9;
    const bool isDry = moistP < soilThreshold; // Use given SOIL_TOPIC
    const bool isRaining = weather.indexOf("Rain") != -1;
    const bool isTooSunny = lux > SUNLIGHT_THRESHOLD;
    static bool lastCommand = false;
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
    }else{
      Serial.print(mqtt.state()); Serial.println(" retry"); delay(2000);
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned len) {
  String msg;
  for (uint32_t i = 0; i < len; i++) msg += (char)payload[i];
  msg.trim(); msg.toUpperCase();
  Serial.println("CMD from MQTT: " + msg);
  // If MQTT message is an integer, set soil threshold to that value
  bool isInt = true;
  for (uint32_t i = 0; i < msg.length(); ++i) {
    if (!isDigit(msg[i])) { isInt = false; break; }
  }
   // Default threshold
  if (isInt && msg.length() > 0) {
    soilThreshold = msg.toFloat();
    Serial.printf("Soil threshold set to %.1f\n", soilThreshold);
    return;
  }
  if (msg)
  if (msg != "TRUE" && msg != "FALSE") return;
  LoRa.idle();
  if (LoRa.beginPacket() && LoRa.print("CMD:" + msg) && LoRa.endPacket()) {
    Serial.println("sent " + msg);
  } else {
    Serial.println("LoRa packet send failed");
  }
  LoRa.receive();
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

void drawOLED(){
  static uint32_t last=0; if(millis()-last<OLED_INTERVAL) return; last=millis();
  oled.fillRect(0,18,128,46,SSD1306_BLACK);
  oled.setCursor(0,18); oled.setTextSize(1);
  oled.printf("Weather: %s\n",weather.c_str());
  oled.printf("Temp: %.1fC\n",tempC);
  oled.printf("Hum:  %.1f%%\n",humP);
  oled.printf("Light: %.1f\n",lux);
  oled.printf("Moist: %.0f%%",moistP); oled.display();
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