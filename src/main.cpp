#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/* ---------------- LoRa pins ---------------- */
const int LORA_CS   = 5;
const int LORA_RST  = 14;
const int LORA_DIO0 = 26;

/* ---------------- OLED --------------------- */
Adafruit_SSD1306 oled(128, 64, &Wire, -1);

/* ---------------- ISR flags ---------------- */
volatile bool packetReady = false;

/* ---------------- helpers ------------------ */
static float extract(const String &src, const String &tag) {
  int i = src.indexOf(tag);
  if (i == -1) return NAN;
  i += tag.length();
  int j = src.indexOf(',', i);
  if (j == -1) j = src.length();
  return src.substring(i, j).toFloat();
}

/* ISR: only flag; parsing is done in loop() */
void onReceive(int len) {
  if (len) packetReady = true;
}

void setup() {
  Serial.begin(115200);

  Wire.begin(); Wire.setClock(400000);
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled.setTextColor(SSD1306_WHITE);
  oled.clearDisplay(); oled.println("LoRa OLED RX"); oled.display();

  SPI.begin();
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(433E6)) { Serial.println("LoRa init failed"); while (true); }
  LoRa.setSyncWord(0xA5); LoRa.enableCrc();
  LoRa.onReceive(onReceive); LoRa.receive();

  Serial.println("Receiver ready");
}

/* -------- runtime vars -------- */
float tC = NAN, hP = NAN, lux = NAN, moist = NAN;
uint32_t lastDraw = 0;

void loop() {

  /* ---- handle packet if ISR flagged ---- */
  if (packetReady) {
    packetReady = false;               // clear flag
    String raw;
    while (LoRa.available()) {
      char c = (char)LoRa.read();
      if (c >= 32 && c <= 126) raw += c;
    }

    float t  = extract(raw, "Temp:");
    float h  = extract(raw, "Hum:");
    if (isnan(h)) h = extract(raw, "Hm:");
    float l  = extract(raw, "Light level:");
    if (isnan(l)) l = extract(raw, "Lux:");
    if (isnan(l)) l = extract(raw, "Lx:");
    float m  = extract(raw, "Moisture:");

    if (!isnan(t) && !isnan(h) && !isnan(l) && !isnan(m)) {
      tC = t; hP = h; lux = l; moist = m;
      Serial.printf("Pkt: T=%.1f  H=%.1f  L=%.1f  M=%2.0f%%\n",
                    t, h, l, m);
    }
    LoRa.receive();                    // re-arm RX
  }

  /* ---- OLED refresh every 250 ms ---- */
/* ---- OLED refresh every 250 ms ---- */
if (millis() - lastDraw >= 250) {
  lastDraw = millis();

  oled.clearDisplay();
  oled.setCursor(0, 0);

  oled.setTextSize(1);  oled.print("Temp:");
  oled.setTextSize(2);  oled.printf(" %.1fC\n", tC);

  oled.setTextSize(1);  oled.print("Hum:");
  oled.setTextSize(2);  oled.printf("  %.1f%%\n", hP);

  oled.setTextSize(1);
  oled.printf("Light: %.1f\n", lux);    // ← line 3
  oled.printf("Moist: %2.0f%%", moist); // ← line 4

  oled.display();
}

}
