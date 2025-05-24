/*****************************************************************
 * AGROSENSE - Arduino Field Node
 * 
 * This node is responsible for:
 * 1. Environmental monitoring (temperature, humidity, light, rain)
 * 2. Soil moisture sensing
 * 3. Valve control for irrigation
 * 4. Two-way LoRa communication with ESP32 gateway
 * 
 * Communication Protocol:
 * TX Format: "Weather:{CLEAR|RAINING}|Temp:{°C}|Hum:{%}|Light level:{0-10}|Moisture:{0-100}|Valve:{OPEN|CLOSE}"
 * RX Format: "CMD:{TRUE|FALSE}" - TRUE=open valve, FALSE=close valve
 * 
 * Features:
 * - Efficient radio switching between RX/TX modes
 * - Optimized LoRa parameters for reliability
 * - Automated valve control based on commands
 * - Calibrated sensor readings
 * 
 * Author: [Your Name]
 * Last Updated: May 2025
 *****************************************************************/

/* ───── Arduino Libraries ───── */
#include <SPI.h>
#include <LoRa.h>
#include <DHT.h>
#include <Servo.h>

/* ───── Pin map (Mega 2560) ───── */
constexpr uint8_t PIN_DHT   = 7;       // DHT11 
constexpr uint8_t PIN_RAIN  = 4;       // rain sensor
constexpr uint8_t PIN_LDR   = A0;      // light sensor
constexpr uint8_t PIN_SOIL  = A1;      // soil moisture
constexpr uint8_t PIN_SERVO = 8;       // valve servo

constexpr uint8_t L_CS   = 53;         // LoRa NSS
constexpr uint8_t L_RST  = 9;          // LoRa RESET
constexpr uint8_t L_DIO0 = 3;          // LoRa DIO0/IRQ

/* ───── LoRa settings ───── */
constexpr long     RF_FREQ   = 433E6;  //433 MHz
constexpr uint8_t  SYNC_WORD = 0xA5;
constexpr uint32_t SEND_INTERVAL = 10000;  // 10 seconds for TX

/* ───── Objects ───── */
DHT   dht(PIN_DHT, DHT11);
Servo valve;

/* ───── State tracking ───── */
enum RadioState { RECEIVING, TRANSMITTING };
RadioState radioState = RECEIVING;  //RX mode by default, only switch to TX when needed to send data

String valveState = "CLOSE";

/* ───── Helpers ───── */
float readLight();    //light sensor
int   readMoist();    //soil moisture
bool  isRaining();    //rain sensor
void  switchToReceive(); 
void  switchToTransmit();

/* ───── Setup ───── */
void setup() {
  Serial.begin(9600);
  while (!Serial) delay(100); 

  dht.begin();
  analogReference(DEFAULT); //set ADC to default ref voltage of 5V

  valve.attach(PIN_SERVO);
  valve.write(0);  // closed

  // Configure DIO0 pin as input with pullup
  pinMode(L_DIO0, INPUT_PULLUP);

  /* LoRa init */
  SPI.begin();
  LoRa.setPins(L_CS, L_RST, L_DIO0);
  LoRa.setSPIFrequency(4E6);  // Increased SPI speed -- 4MHz
  
  //verify LoRa communication
  if (!LoRa.begin(RF_FREQ)) {
    Serial.println(F("LoRa init failed"));
    while (true) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(200);
    }
  }
  
  LoRa.setSyncWord(SYNC_WORD);    //differentiate network from other nearby networks
  LoRa.enableCrc();               //verify data integrity
  LoRa.setSpreadingFactor(7);    // control data rate & range -- slightly faster, shorter range, less power
  LoRa.setSignalBandwidth(125E3); // Standard bandwidth
  LoRa.setCodingRate4(5);        // Lower coding rate for speed
  
  switchToReceive();
  Serial.println(F("Valve node ready"));
}

/* ───── Main Loop ───── */
void loop() {
    static uint32_t lastSend = 0;

    /* ---------- Command Reception (RX Mode) ---------- 
     * The node spends most of its time in RX mode listening for valve commands
     * from the gateway. Commands are processed immediately upon receipt.
     */
    if (radioState == RECEIVING) {
        int packetSize = LoRa.parsePacket();  
    /*Check if packet has been received,
    and if so extract the message/command*/
    if (packetSize > 0) {
      String cmd = "";
      while (LoRa.available()) {
        cmd += (char)LoRa.read();
      }
      cmd.trim();
      cmd.toUpperCase();
      
      Serial.println("RX → " + cmd + " (RSSI: " + String(LoRa.packetRssi()) + ")"); //track the RX message recieved

      if (cmd == "CMD:TRUE") {
        valve.write(90);  //open the valve
        valveState = "OPEN";
        Serial.println("Valve OPEN");
      } else if (cmd == "CMD:FALSE") {
        valve.write(0);   //close the valve
        valveState = "CLOSE";
        Serial.println("Valve CLOSE");
      }
    }
  }

  /* ---------- Sensor Data Transmission (TX Mode) ----------
   * Every SEND_INTERVAL (10s), the node:
   * 1. Switches to TX mode
   * 2. Collects readings from all sensors
   * 3. Formats and sends data packet
   * 4. Returns to RX mode
   * 
   * The transmission time is minimized to avoid missing incoming commands
   */
  if (millis() - lastSend >= SEND_INTERVAL) {
    lastSend = millis();
    
    switchToTransmit(); //transmit sensor data every 10s to the esp32 over LoRa
    
    float h = dht.readHumidity();     //read humidity data
    float t = dht.readTemperature();  //read temperature data
    
    //validate the dht11 data received
    if (isnan(h) || isnan(t)) {
      Serial.println(F("DHT error"));
      switchToReceive();
      return;
    }

    //capture the data received from the other sensors
    float light = readLight();
    int   moist = readMoist();
    String weather = isRaining() ? "Raining" : "Clear"; //display "Clear" for periods of no rain

    //message sent to the esp32 detailing the data received from all sensors including the current state of the valve
    String pkt = "Weather:" + weather +
                 "|Temp:" + String(t, 1) +
                 "|Hum:" + String(h, 1) +
                 "|Light level:" + String(light, 1) +
                 "|Moisture:" + String(moist) +
                 "|Valve:" + valveState;

    LoRa.beginPacket(); //switch to TX mode
    LoRa.print(pkt);    //send LoRa msg
    LoRa.endPacket();   
    
    Serial.println("TX → " + pkt);  //track the TX msg
    
    /*Purpose: We want to spend minimum time for TX while ensuring high reliability so
    we can receive data during the remaining 99.9% of the time to avoid missing packets*/
    // Wait for transmission to complete (estimate based on packet size)
    int txTime = pkt.length() + 50;  // roughly 1ms per byte +50ms buffer, ex: 75ms for 25 byte msg pkt
    delay(txTime);
    
    // Small delay before switching back to RX
    delay(100);
    switchToReceive();
  }
  
  delay(10);  // Prevent excessive CPU usage while maintaining responsiveness
}

/* ───── Radio Control Functions ───── */
/**
 * Switch radio to receive mode (RX)
 * This is the default state where the node listens for valve commands
 */
void switchToReceive() {
  if (radioState != RECEIVING) {
    LoRa.receive();
    radioState = RECEIVING;
    delay(10);  // Allow mode switch to settle
  }
}

/**
 * Switch radio to transmit mode (TX)
 * Used briefly every SEND_INTERVAL to send sensor data
 */
void switchToTransmit() {
  if (radioState != TRANSMITTING) {
    LoRa.idle();  // Stop receiving first
    radioState = TRANSMITTING;
    delay(10);    // Allow mode switch to settle
  }
}

/* ───── Sensor Functions ───── */
/**
 * Read light level from LDR sensor
 * @return float Light level on a 0-10 scale (0=dark, 10=bright)
 * Values are calibrated for typical ambient light conditions
 */
float readLight() {
  int adc = analogRead(PIN_LDR);
  float level = mapFloat(ADC, 1200.0, -100.0, 0.0, 10.0); //calibrated the light sensor to show realistic values for ambient lighting
  return constrain(level, 0.0, 10.0); //ensure the light level is shown from 0 - 10 scale
}

/**
 * Read soil moisture percentage
 * @return int Moisture level 0-100% (0=dry, 100=saturated)
 * ADC values calibrated for typical soil conditions
 */
int readMoist() {
  int adc = analogRead(PIN_SOIL);
  int moist = map(adc, 1023, 300, 0, 100);
  return constrain(moist, 0, 100);
}

/**
 * Check rain sensor state
 * @return bool True if rain is detected, false otherwise
 * Sensor is active-low (LOW = rain detected)
 */
bool isRaining() {
  return digitalRead(PIN_RAIN) == LOW;
}

/* ───── Utility Functions ───── */
/**
 * Maps a float value from one range to another
 * Similar to Arduino's map() but supports floating point
 * @param x Input value to map
 * @param in_min Input range minimum
 * @param in_max Input range maximum
 * @param out_min Output range minimum
 * @param out_max Output range maximum
 * @return float Mapped value in output range
 */
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
