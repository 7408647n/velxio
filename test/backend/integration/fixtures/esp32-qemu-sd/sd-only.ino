// Bus matrix, scenarios c (Uno) and e (ESP32 DevKit): a microSD card alone.
//
// Mounts the card and reads /data.txt back on every loop pass. A card that
// stops answering after Stop/Run or Reset shows up as SD FAIL with the loop
// still alive.
//
// Lines the harness reads (printed from loop()):
//   LOOP <n>
//   SD PASS begin=1 | SD FAIL begin=<0|1>
#include <SPI.h>
#include <SD.h>

#if defined(ESP32)
#define SD_CS 5
#else
#define SD_CS 4
#endif

static const char EXPECT[] = "VELXIO-BUS-MATRIX-7F3A";
static bool sdUp = false;

static bool readBack() {
  File f = SD.open("/data.txt");
  if (!f) return false;
  char buf[32];
  uint8_t k = 0;
  while (f.available() && k < sizeof(buf) - 1) buf[k++] = f.read();
  buf[k] = 0;
  f.close();
  return strncmp(buf, EXPECT, sizeof(EXPECT) - 1) == 0;
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println(F("BOOT sd-only"));
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  sdUp = SD.begin(SD_CS);
  Serial.println(F("SETUP DONE"));
}

void loop() {
  static uint32_t n = 0;
  Serial.print(F("LOOP "));
  Serial.println(++n);
  if (!sdUp) sdUp = SD.begin(SD_CS);
  bool ok = sdUp && readBack();
  Serial.print(F("SD "));
  Serial.print(ok ? F("PASS") : F("FAIL"));
  Serial.print(F(" begin="));
  Serial.println(sdUp ? 1 : 0);
  delay(250);
}
