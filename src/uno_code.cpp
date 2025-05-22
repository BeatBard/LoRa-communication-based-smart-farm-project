#include <SPI.h>
#include <LoRa.h>
#include <DHT.h>
#include <Servo.h>

/* ───── Pin map (Mega 2560) ───── */
constexpr uint8_t PIN_DHT   = 3;       // DHT11 signal
constexpr uint8_t PIN_RAIN  = 4;       // rain sensor (active-low)
constexpr uint8_t PIN_LDR   = A0;      // light sensor
constexpr uint8_t PIN_SOIL  = A1;      // soil moisture
constexpr uint8_t PIN_SERVO = 8;       // valve servo

constexpr uint8_t L_CS   = 53;         // LoRa NSS
constexpr uint8_t L_RST  = 9;          // LoRa RESET
constexpr uint8_t L_DIO0 = 2;          // LoRa DIO0/IRQ

/* ───── LoRa settings ───── */
constexpr long     RF_FREQ   = 433E6;
constexpr uint8_t  SYNC_WORD = 0xA5;
constexpr uint32_t SEND_INTERVAL = 2000;   // ms

/* ───── Objects ───── */
DHT   dht(PIN_DHT, DHT11);
Servo valve;

/* ───── ISR flag ───── */
volatile bool packetReady = false;
void onRx() { packetReady = true; }    // AVR ISR (no attribute needed)

/* ───── Helpers ───── */
float readLight();
int   readMoist();
bool  isRaining();

/* ───── Setup ───── */
void setup() {
  Serial.begin(9600);

  dht.begin();
  analogReference(DEFAULT);            // 5 V ADC

  valve.attach(PIN_SERVO);
  valve.write(0);                      // closed

  /* LoRa init */
  SPI.begin();                         // MOSI=51 MISO=50 SCK=52
  LoRa.setPins(L_CS, L_RST, L_DIO0);
  LoRa.setSPIFrequency(1E6);
  if (!LoRa.begin(RF_FREQ)) {
    Serial.println(F("LoRa init failed")); while (true);
  }
  LoRa.setSyncWord(SYNC_WORD);
  LoRa.enableCrc();
  LoRa.receive();
  attachInterrupt(digitalPinToInterrupt(L_DIO0), onRx, RISING);

  Serial.println(F("Valve node ready"));
}

/* ───── Loop ───── */
void loop() {
  static uint32_t lastSend = 0;

  /* ---------- periodic TX ---------- */
  if (millis() - lastSend >= SEND_INTERVAL) {
    lastSend = millis();

    float h = dht.readHumidity();
    float t = dht.readTemperature();
    if (isnan(h) || isnan(t)) { Serial.println(F("DHT error")); return; }

    float light = readLight();
    int   moist = readMoist();
    String weather = isRaining() ? "Raining" : "Clear";

    String pkt =  "Weather:"      + weather +
                  "|Temp:"        + String(t, 1) +
                  "|Hum:"         + String(h, 1) +
                  "|Light level:" + String(light, 1) +
                  "|Moisture:"    + String(moist);

    LoRa.beginPacket();
    LoRa.print(pkt);
    LoRa.endPacket();
    Serial.println("TX → " + pkt);
  }

  /* ---------- command RX ---------- */
  if (packetReady) {
    packetReady = false;
    if (LoRa.parsePacket()) {
      String cmd = LoRa.readString();
      cmd.trim(); cmd.toUpperCase();
      Serial.println("RX → " + cmd);

      if (cmd == "CMD:ON")  { valve.write(90);  Serial.println("Valve OPEN"); }
      if (cmd == "CMD:OFF") { valve.write(0);   Serial.println("Valve CLOSE");}
    }
    LoRa.receive();                             // back to RX
  }
}

/* ───── Sensor functions ───── */
float readLight() {
  int adc = analogRead(PIN_LDR);                // 0–1023
  float level = (1023 - adc) * 10.0 / 1023.0;   // scale 0-10
  return constrain(level, 0.0, 10.0);
}
int readMoist() {
  int adc = analogRead(PIN_SOIL);               // 0–1023
  int moist = map(adc, 1023, 300, 0, 100);      // calibrate dry 300, wet 700
  return constrain(moist, 0, 100);
}
bool isRaining() { return digitalRead(PIN_RAIN) == LOW; }