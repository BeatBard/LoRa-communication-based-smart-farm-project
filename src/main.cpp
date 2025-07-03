/*****************************************************************
 * AGROSENSE - ESP32 LoRa Agricultural Automation Gateway
 * 
 * This gateway serves as a bridge between:
 * 1. LoRa-connected field sensors/actuators
 * 2. WiFi/MQTT for cloud connectivity
 * 3. Local OLED display for status monitoring
 * 
 * Features:
 * - Two-way LoRa communication with field nodes
 * - MQTT integration for remote monitoring and control
 * - Real-time environmental monitoring (temp, humidity, light, soil)
 * - Automatic/Manual irrigation control based on conditions
 * - OLED display with custom icons for visual feedback
 * - NTP time synchronization
 * 
 * Author: Nimesh, Pamith and Udith
 * Last Updated: May 2025
 *****************************************************************/

/* -------------------- Library Includes -------------------- */
#include <Arduino.h>
#include <SPI.h>          // For LoRa and OLED communication
#include <LoRa.h>         // LoRa radio functionality
#include <WiFi.h>         // WiFi connectivity
#include <PubSubClient.h> // MQTT client

// Display and Graphics
#include <Wire.h>             // I2C for OLED
#include <Adafruit_GFX.h>     // Graphics core library
#include <Adafruit_SSD1306.h> // OLED display driver

// Utilities
#include <ArduinoJson.h>      // JSON parsing/creation
#include <time.h>             // Time functions
#include <sys/time.h>         // System time utilities

/* -------------------- Custom Icons (8x8 pixels) -------------------- */
// Weather and sensor emojis (8x8 pixels)
const uint8_t temp_emoji[] = {
    0b00001000,
    0b00001100,
    0b00001000,
    0b00001100,
    0b00001000,
    0b00011100,
    0b00011100,
    0b00001000
};

const uint8_t humid_emoji[] = {
    0b00001000,
    0b00001000,
    0b00011100,
    0b00011100,
    0b00111110,
    0b00111110,
    0b01111111,
    0b00111110
};

const uint8_t moist_emoji[] = {
    0b00000100,
    0b00001100,
    0b00011100,
    0b00111100,
    0b01111000,
    0b00110000,
    0b00100000,
    0b00000000
};

const uint8_t sun_emoji[] = {
    0b00001000,
    0b00101010,
    0b00011100,
    0b01111110,
    0b00011100,
    0b00101010,
    0b00001000,
    0b00000000
};

const uint8_t rain_emoji[] = {
    0b01110000,
    0b11111100,
    0b01110010,
    0b00001001,
    0b00010010,
    0b00100100,
    0b01001000,
    0b10010000
};

/* -------------------- Network Configuration -------------------- */
// WiFi Settings
const char* WIFI_SSID = "Redmi Note 11";
const char* WIFI_PASS = "lfbn7105";

// MQTT Configuration
const char* MQTT_HOST = "test.mosquitto.org";
const uint16_t MQTT_PORT = 1883;

// MQTT Topics
const char* PUB_TOPIC = "IoT-G9";          // Main sensor data publication
const char* VAL_TOPIC = "IoT-G9/valve";     // Valve status updates
const char* CMD_TOPIC = "IoT-G9/cmd";       // Command reception
const char* SOIL_TOPIC = "IoT-G9/soil";     // Soil threshold settings
const char* MODE_TOPIC = "IoT-G9/mode";     // Operation mode control

/* -------------------- System Configuration -------------------- */
// NTP Settings
const char* NTP_SERVER = "pool.ntp.org";
const long GMT_OFFSET_SEC = 5 * 3600 + 30 * 60;  // Sri Lanka GMT+5:30
const int DAYLIGHT_OFFSET_SEC = 0;

// LoRa Radio Configuration
constexpr long RF_FREQ = 433E6;             // 433 MHz frequency band
constexpr uint8_t SYNC_WORD = 0xA5;         // LoRa sync word
constexpr gpio_num_t L_CS = GPIO_NUM_5;     // Chip select
constexpr gpio_num_t L_RST = GPIO_NUM_14;   // Reset
constexpr gpio_num_t L_DIO0 = GPIO_NUM_26;  // Interrupt

// OLED Display Settings
Adafruit_SSD1306 oled(128, 64, &Wire, -1);  // 128x64 OLED
constexpr uint32_t OLED_INTERVAL = 250;      // Display refresh interval

// GPIO Configuration
constexpr gpio_num_t LED_PIN = GPIO_NUM_2;   // Status LED

/* -------------------- Global Variables -------------------- */
// Communication Objects
WiFiClient net;
PubSubClient mqtt(net);

// System State
volatile bool packetReady = false;          // LoRa packet received flag
bool isManualMode = true;                  // Operation mode flag
bool lastCommand = false;                   // Last valve command state
static float soilThreshold = 30.0;          // Moisture threshold

// Sensor Data
String weather = "N/A";                     // Current weather condition
String valve = "N/A";                       // Valve state
float tempC = NAN;                         // Temperature in Celsius
float humP = NAN;                          // Humidity percentage
float lux = NAN;                           // Light level
float moistP = NAN;                        // Soil moisture percentage

/* -------------------- Function Prototypes -------------------- */
// Network Functions
void connectWiFi();
void connectMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int len);

// Data Processing
void publishJSON();
String extractStr(const String&, const String&);
float extractFloat(const String&, const String&);
String getTimestamp();
char weatherIconAscii(const String&, float);

// Display Functions
void drawOLED();

// LoRa Interrupt Handler
void IRAM_ATTR onPacketISR(int) { packetReady = true; }

void setup() {
  // Initialize Serial communication
  Serial.begin(115200);
  
  // Configure LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Initialize I2C for OLED
  Wire.begin();
  Wire.setClock(400000);
  
  // Initialize OLED display
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(2);
  oled.clearDisplay();
  oled.println("Aulak nane");
  oled.display();

  // Setup WiFi and NTP time sync
  connectWiFi();
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("NTP Time Sync Failed");
  } else {
    Serial.println(&timeinfo, "NTP Time: %Y-%m-%d %H:%M:%S");
  }

  // Configure MQTT
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);

  // Initialize LoRa radio
  SPI.begin();
  LoRa.setPins(L_CS, L_RST, L_DIO0);
  
  if(!LoRa.begin(RF_FREQ)) {
    Serial.println("LoRa initialization failed");
    while(true);  // Halt if LoRa init fails
  }
  
  // Configure LoRa parameters
  LoRa.setSyncWord(SYNC_WORD);
  LoRa.enableCrc();
  LoRa.onReceive(onPacketISR);
  LoRa.receive();
  
  Serial.println("Gateway ready");
}

void loop() {
  // Maintain network connections
  if(WiFi.status()!=WL_CONNECTED) connectWiFi();
  if(!mqtt.connected()) connectMQTT();
  mqtt.loop();

  // Process LoRa packets when received
  if(packetReady){
    packetReady = false;
    String raw;
    
    // Read and validate incoming LoRa data
    while(LoRa.available()){
      char c = char(LoRa.read());
      if(c >= 32 && c <= 126) raw += c;  // Only accept printable ASCII
    }

    // Extract sensor data from LoRa packet
    weather = extractStr(raw, "Weather:");
    tempC   = extractFloat(raw, "Temp:");
    humP    = extractFloat(raw, "Hum:"); 
    if(isnan(humP)) humP = extractFloat(raw, "Hm:");
    
    // Handle multiple possible light level tags
    lux = extractFloat(raw, "Light level:");
    if(isnan(lux)) lux = extractFloat(raw, "Lux:");
    if(isnan(lux)) lux = extractFloat(raw, "Lx:");
    
    moistP = extractFloat(raw, "Moisture:");
    valve  = extractStr(raw, "Valve:");
    
    // Publish sensor data to MQTT and update display
    mqtt.publish(VAL_TOPIC, valve.c_str());
    Serial.printf("RX: %s | T %.1f | H %.1f | L %.1f | M %.0f%% | V %s\n",
                  weather.c_str(), tempC, humP, lux, moistP, valve.c_str());
    publishJSON();
    drawOLED();

    // Automated valve control logic (when in auto mode)
    if (!isManualMode) {
      const float SUNLIGHT_THRESHOLD = 8.5;  // Lux threshold for too sunny
      
      // Check environmental conditions
      const bool isDry = moistP < soilThreshold;
      String weatherUpper = weather;
      weatherUpper.toUpperCase();
      const bool isRaining = weather.length() > 0 && weatherUpper.indexOf("RAIN") != -1;
      const bool isTooSunny = lux > SUNLIGHT_THRESHOLD;
      
      // Determine if valve state should change
      bool newCommand = (isDry && !isRaining && !isTooSunny);
      
      if (newCommand != lastCommand) {
        lastCommand = newCommand;
        String cmd = newCommand ? "TRUE" : "FALSE";
        
        // Configure LoRa for transmission
        LoRa.idle();
        const int maxRetries = 3;
        
        // Send command multiple times for reliability
        for (int i = 0; i < maxRetries; ++i) {
          if (LoRa.beginPacket() && LoRa.print("CMD:" + cmd) && LoRa.endPacket()) {
            Serial.println("Auto CMD sent (burst " + String(i + 1) + "): " + cmd);
          } else {
            Serial.println("LoRa CMD Send Failed (burst " + String(i + 1) + ")");
          }
          delay(50);  // Small delay between retries to avoid collisions
        }
        LoRa.receive();
      }
    }
    
    // Return to listening mode
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
  // Convert payload to string and normalize format
  String msg;
  for (uint32_t i = 0; i < len; i++) msg += (char)payload[i];
  msg.trim(); 
  msg.toUpperCase();  // Normalize to uppercase for case-insensitive comparison
  
  /*-------------------- Mode Control Handler --------------------*/
  if (strcmp(topic, MODE_TOPIC) == 0) {
    // FALSE = Manual Mode, TRUE = Auto Mode
    if (msg == "FALSE") {
      isManualMode = true;
      Serial.println("Switching to manual mode");
    } else if (msg == "TRUE") {
      isManualMode = false;
      Serial.println("Switching to auto mode");
    }
    return;
  }
  
  /*-------------------- Soil Threshold Handler --------------------*/
  if (strcmp(topic, SOIL_TOPIC) == 0) {
    // Validate that message contains only digits
    bool isInt = true;
    for (uint32_t i = 0; i < msg.length(); ++i) {
      if (!isDigit(msg[i])) { 
        isInt = false; 
        break; 
      }
    }
    // Update threshold if valid number received
    if (isInt && msg.length() > 0) {
      soilThreshold = msg.toFloat();
      Serial.printf("Soil threshold set to %.1f\n", soilThreshold);
      return;
    }
  }

  /*-------------------- Valve Control Handler --------------------*/
  if (strcmp(topic, CMD_TOPIC) == 0 && isManualMode) {
    if (msg == "TRUE" || msg == "FALSE") {
      Serial.println("Manual CMD from MQTT: " + msg);
      
      // Update last command state
      lastCommand = (msg == "TRUE");
      
      // Configure LoRa for transmission
      LoRa.idle();
      const int maxRetries = 3;  // Number of transmission attempts
      
      // Send command multiple times for reliability
      for (int i = 0; i < maxRetries; ++i) {
        if (LoRa.beginPacket() && LoRa.print("CMD:" + msg) && LoRa.endPacket()) {
          Serial.println("Manual CMD sent (burst " + String(i + 1) + "): " + msg);
        } else {
          Serial.println("LoRa CMD send failed (burst " + String(i + 1) + ")");
        }
        delay(50);  // Small delay between retries to avoid collisions
      }
      
      // Return to receiving mode
      LoRa.receive();
    }
  }
}

void publishJSON(){
  JsonDocument doc;
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

  // Title
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(35, 0);
  oled.println("AGROSENSE");
  oled.drawLine(0, 9, 128, 9, SSD1306_WHITE);  // Underline

  // --- Icons and Readings ---
  oled.setTextSize(1);

  // Temperature reading with thermometer emoji
  oled.drawBitmap(2, 17, temp_emoji, 8, 8, SSD1306_WHITE);
  oled.setCursor(12, 18);
  oled.printf("%.1fC", tempC);

  // Humidity reading with droplet emoji
  oled.drawBitmap(64, 17, humid_emoji, 8, 8, SSD1306_WHITE);
  oled.setCursor(74, 18);
  oled.printf("%.1f%%", humP);

  // Light level with sun emoji
  oled.drawBitmap(2, 30, sun_emoji, 8, 8, SSD1306_WHITE);
  oled.setCursor(12, 31);
  oled.printf("%.1f lx", lux);

  // Moisture reading with moisture emoji
  oled.drawBitmap(64, 30, moist_emoji, 8, 8, SSD1306_WHITE);
  oled.setCursor(74, 31);
  oled.printf("%.0f%%", moistP);
  // Weather status with dynamic weather icon
  oled.setCursor(0, 44);
  oled.printf("Weather: %s", weather.c_str());
  
  // Draw weather icon based on conditions
  String weatherUpper = weather;
  weatherUpper.toUpperCase();
  if (weatherUpper.indexOf("RAIN") != -1) {
    oled.drawBitmap(100, 42, rain_emoji, 8, 8, SSD1306_WHITE);
  } else if (lux > 9) {
    oled.drawBitmap(100, 42, sun_emoji, 8, 8, SSD1306_WHITE);
  } else {
    // Default cloud-like pattern for other conditions
    oled.fillCircle(104, 46, 3, SSD1306_WHITE);
    oled.fillCircle(100, 46, 2, SSD1306_WHITE);
    oled.fillCircle(108, 46, 2, SSD1306_WHITE);
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