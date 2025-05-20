#include <SPI.h>
#include <LoRa.h>

const int csPin = 5;
const int resetPin = 14;
const int irqPin = 26;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  LoRa.setPins(csPin, resetPin, irqPin);

  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa init failed!");
    while (1);
  }

  Serial.println("LoRa receiver started");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String message = "";

    while (LoRa.available()) {
      char c = (char)LoRa.read();

      // Stop reading at newline (end of message)
      if (c == '\n') break;

      // Only accept printable characters
      if (c >= 32 && c <= 126) {
        message += c;
      }
    }

    // Optional validation
    if (message.startsWith("Temp:") && message.indexOf("Hum:") != -1) {
      Serial.print("Clean message: ");
      Serial.println(message);
    } else {
      Serial.println("Discarded noisy or malformed packet");
    }
  }
}
