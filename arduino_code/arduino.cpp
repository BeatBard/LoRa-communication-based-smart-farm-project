#include <SPI.h>
#include <LoRa.h>
#include <DHT.h>
#include <Servo.h>

/* ───── Pin map (Mega 2560) ───── */
constexpr uint8_t PIN_DHT   = 7;       // DHT11 signal (moved from pin 3)
constexpr uint8_t PIN_RAIN  = 4;       // rain sensor (active-low)
constexpr uint8_t PIN_LDR   = A0;      // light sensor
constexpr uint8_t PIN_SOIL  = A1;      // soil moisture
constexpr uint8_t PIN_SERVO = 8;       // valve servo

constexpr uint8_t L_CS   = 53;         // LoRa NSS
constexpr uint8_t L_RST  = 9;          // LoRa RESET
constexpr uint8_t L_DIO0 = 3;          // LoRa DIO0/IRQ (changed to pin 3)

/* ───── LoRa settings ───── */
constexpr long     RF_FREQ   = 433E6;
constexpr uint8_t  SYNC_WORD = 0xA5;
constexpr uint32_t SEND_INTERVAL = 10000;  // 10 seconds for TX

/* ───── Objects ───── */
DHT   dht(PIN_DHT, DHT11);
Servo valve;

/* ───── State tracking ───── */
enum RadioState { RECEIVING, TRANSMITTING };
RadioState radioState = RECEIVING;

/* ───── Helpers ───── */
float readLight();
int   readMoist();
bool  isRaining();
void  switchToReceive();
void  switchToTransmit();

/* ───── Setup ───── */
void setup() {
  Serial.begin(9600);
  while (!Serial) delay(100);  // Wait for serial on Leonardo/Micro

  dht.begin();
  analogReference(DEFAULT);

  valve.attach(PIN_SERVO);
  valve.write(0);  // closed

  // Configure DIO0 pin as input with pullup
  pinMode(L_DIO0, INPUT_PULLUP);

  /* LoRa init */
  SPI.begin();
  LoRa.setPins(L_CS, L_RST, L_DIO0);
  LoRa.setSPIFrequency(4E6);  // Increased SPI speed
  
  if (!LoRa.begin(RF_FREQ)) {
    Serial.println(F("LoRa init failed"));
    while (true) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(200);
    }
  }
  
  LoRa.setSyncWord(SYNC_WORD);
  LoRa.enableCrc();
  LoRa.setSpreadingFactor(7);    // Faster data rate
  LoRa.setSignalBandwidth(125E3); // Standard bandwidth
  LoRa.setCodingRate4(5);        // Lower coding rate for speed
  
  switchToReceive();
  Serial.println(F("Valve node ready"));
}

/* ───── Loop ───── */
void loop() {
  static uint32_t lastSend = 0;

  /* ---------- Check for incoming packets ---------- */
  if (radioState == RECEIVING) {
    int packetSize = LoRa.parsePacket();
    if (packetSize > 0) {
      String cmd = "";
      while (LoRa.available()) {
        cmd += (char)LoRa.read();
      }
      cmd.trim();
      cmd.toUpperCase();
      
      Serial.println("RX → " + cmd + " (RSSI: " + String(LoRa.packetRssi()) + ")");

      if (cmd == "CMD:TRUE") {
        valve.write(90);
        Serial.println("Valve OPEN");
      } else if (cmd == "CMD:FALSE") {
        valve.write(0);
        Serial.println("Valve CLOSE");
      }
    }
  }

  /* ---------- Periodic TX ---------- */
  if (millis() - lastSend >= SEND_INTERVAL) {
    lastSend = millis();
    
    switchToTransmit();
    
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    
    if (isnan(h) || isnan(t)) {
      Serial.println(F("DHT error"));
      switchToReceive();
      return;
    }

    float light = readLight();
    int   moist = readMoist();
    String weather = isRaining() ? "Raining" : "Clear";

    String pkt = "Weather:" + weather +
                 "|Temp:" + String(t, 1) +
                 "|Hum:" + String(h, 1) +
                 "|Light level:" + String(light, 1) +
                 "|Moisture:" + String(moist);

    LoRa.beginPacket();
    LoRa.print(pkt);
    LoRa.endPacket();
    
    Serial.println("TX → " + pkt);
    
    // Wait for transmission to complete (estimate based on packet size)
    // Rough calculation: ~1ms per byte at SF7, BW125kHz
    int txTime = pkt.length() + 50;  // +50ms buffer
    delay(txTime);
    
    // Small delay before switching back to RX
    delay(100);
    switchToReceive();
  }
  
  delay(10);  // Small delay to prevent tight loop
}

/* ───── Radio control functions ───── */
void switchToReceive() {
  if (radioState != RECEIVING) {
    LoRa.receive();
    radioState = RECEIVING;
    delay(10);  // Allow mode switch to settle
  }
}

void switchToTransmit() {
  if (radioState != TRANSMITTING) {
    LoRa.idle();  // Stop receiving first
    radioState = TRANSMITTING;
    delay(10);    // Allow mode switch to settle
  }
}

/* ───── Sensor functions ───── */
float readLight() {
  int adc = analogRead(PIN_LDR);
  float level = (1023 - adc) * 10.0 / 1023.0;
  return constrain(level, 0.0, 10.0);
}

int readMoist() {
  int adc = analogRead(PIN_SOIL);
  int moist = map(adc, 1023, 300, 0, 100);
  return constrain(moist, 0, 100);
}

bool isRaining() {
  return digitalRead(PIN_RAIN) == LOW;
}
