// CYD ESP32: connect to ESP32-CAM SoftAP and display JPEG frames rotated 90° CW
// Works with ESP32-CAM sketch above (SoftAP "OrangePortal" / "TheCakeIsALie")
// and JPEG frames over TCP port 5000.

#include <WiFi.h>
#include <TFT_eSPI.h>
#include <TJpg_Decoder.h>

// ---------- Wi-Fi (connect to ESP32-CAM AP) ----------
const char* WIFI_SSID = "OrangePortal";
const char* WIFI_PASS = "TheCakeIsALie";

// ESP32-CAM SoftAP default IP is usually 192.168.4.1
const char*   CAM_HOST = "192.168.4.1";
const uint16_t CAM_PORT = 5000;

// TFT + JPEG
TFT_eSPI tft = TFT_eSPI();
WiFiClient camClient;

// JPEG buffer (enough for QVGA JPEG)
static uint8_t* jpgBuf = nullptr;
static const size_t JPG_BUF_SIZE = 80 * 1024;

// Control frame dropping (decode every Nth frame)
static uint32_t frameCounter = 0;
static const uint8_t DECODE_EVERY_N = 1;   // set 2 or 3 to drop frames if needed

// ----- TJpg callback -----
bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap)
{
  if (y >= tft.height()) return 0;
  tft.pushImage(x, y, w, h, bitmap);
  return 1;
}

// ----- Network helpers -----
bool readBytesExact(uint8_t* buf, size_t len, uint32_t timeoutMs = 5000) {
  size_t got = 0;
  uint32_t start = millis();
  while (got < len && (millis() - start) < timeoutMs) {
    int avail = camClient.available();
    if (avail > 0) {
      int readLen = camClient.read(buf + got, len - got);
      if (readLen <= 0) break;
      got += readLen;
    } else {
      delay(1);
      // yield();  // optional on some cores
    }
  }
  return (got == len);
}

bool connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return true;

  tft.fillScreen(TFT_BLACK);
  tft.setCursor(5, 5);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.println("WiFi: connecting...");
  tft.println(WIFI_SSID);

  Serial.print("Connecting WiFi to ");
  Serial.println(WIFI_SSID);

  WiFi.disconnect(true, true);
  delay(100);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(500);
    Serial.print(".");
    tft.print(".");               // simple progress indicator
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi connect failed.");
    tft.println();
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.println("WiFi failed");
    tft.println("Check ESP32-CAM");
    return false;
  }

  Serial.print("WiFi OK, IP: ");
  Serial.println(WiFi.localIP());

  tft.fillScreen(TFT_BLACK);
  tft.setCursor(5, 5);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.println("WiFi connected");
  tft.print("IP: ");
  tft.println(WiFi.localIP());

  return true;
}


bool connectCamera() {
  if (camClient.connected()) return true;

  camClient.stop();
  Serial.printf("Connecting to camera %s:%u\n", CAM_HOST, CAM_PORT);
  if (!camClient.connect(CAM_HOST, CAM_PORT)) {
    Serial.println("Camera connect failed, retrying later...");
    return false;
  }
  camClient.setNoDelay(true);
  Serial.println("Camera connected.");
  return true;
}

// ----- Setup -----
void setup() {
  Serial.begin(115200);
  delay(500);

  // TFT init
  tft.init();
  tft.setRotation(3);       // 90° CW on many CYD boards
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(5, 5);
  tft.println("Connecting...");

  // JPEG decoder
  TJpgDec.setJpgScale(1);   // Camera sends QVGA (320x240)[web:48]
  TJpgDec.setSwapBytes(true);
  TJpgDec.setCallback(tft_output);

  jpgBuf = (uint8_t*)malloc(JPG_BUF_SIZE);
  if (!jpgBuf) {
    Serial.println("Failed to allocate JPEG buffer");
    tft.println("JPG buf alloc fail");
  }

  connectWiFi();
}

// ----- Main loop -----
void loop() {
  if (!jpgBuf) {
    delay(1000);
    return;
  }

  // Ensure Wi-Fi connection; retry if it fails
  if (!connectWiFi()) {
    delay(1000);
    return;
  }

  // Ensure TCP connection to camera; retry if it fails
  if (!connectCamera()) {
    delay(500);
    return;
  }

  // --- Read 4-byte length header ---
  uint8_t header[4];
  if (!readBytesExact(header, 4, 5000)) {
    Serial.println("Header read failed, reconnecting...");
    camClient.stop();    // Force reconnect next loop
    delay(200);
    return;
  }

  size_t jpgLen =
      ((size_t)header[0] << 24) |
      ((size_t)header[1] << 16) |
      ((size_t)header[2] << 8)  |
      ((size_t)header[3]);

  if (jpgLen == 0 || jpgLen > JPG_BUF_SIZE) {
    Serial.printf("Bad JPG len: %u, reconnecting...\n", (unsigned)jpgLen);
    camClient.stop();
    delay(200);
    return;
  }

  // --- Read JPEG payload ---
  if (!readBytesExact(jpgBuf, jpgLen, 5000)) {
    Serial.println("JPEG read failed, reconnecting...");
    camClient.stop();
    delay(200);
    return;
  }

  frameCounter++;

  // Optionally drop some frames to reduce TFT/CPU load
  if (DECODE_EVERY_N > 1 && (frameCounter % DECODE_EVERY_N) != 0) {
    // Skip decoding, but keep reading data to keep TCP flowing
    return;
  }

  uint32_t t = millis();

  // Get JPEG size and center it on the rotated screen
  uint16_t w = 0, h = 0;
  TJpgDec.getJpgSize(&w, &h, jpgBuf, jpgLen);

  int16_t x = (tft.width()  - w) / 2;
  int16_t y = (tft.height() - h) / 2;
  if (x < 0) x = 0;
  if (y < 0) y = 0;

  TJpgDec.drawJpg(x, y, jpgBuf, jpgLen);

  t = millis() - t;
  Serial.printf("Draw time: %u ms\n", (unsigned)t);
}
