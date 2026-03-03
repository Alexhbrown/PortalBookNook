// ESP32-CAM: SoftAP + JPEG over TCP server

#include "esp_camera.h"
#include <WiFi.h>

// ---------- Wi-Fi SoftAP config ----------
const char* AP_SSID = "OrangePortal";
const char* AP_PASS = "TheCakeIsALie";
const uint16_t TCP_PORT = 5000;

// ---------- Camera pin definition (AI-Thinker) ----------
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

WiFiServer server(TCP_PORT);
WiFiClient client;

// ---------- Camera setup ----------
bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // QVGA is 320x240, good for ~15+ fps with JPEG.[web:42]
  config.frame_size   = FRAMESIZE_QVGA;
  config.jpeg_quality = 15;   // 10–20 is typical for streaming
  config.fb_count     = 2;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }

  sensor_t* s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);

  return true;
}

// Send frame with 4‑byte length header (big endian) + JPEG data
bool sendFrame(WiFiClient& c, const uint8_t* data, size_t len) {
  if (!c.connected()) return false;

  uint8_t header[4];
  header[0] = (len >> 24) & 0xFF;
  header[1] = (len >> 16) & 0xFF;
  header[2] = (len >> 8)  & 0xFF;
  header[3] = (len)       & 0xFF;

  if (c.write(header, 4) != 4) return false;
  if (c.write(data, len) != (int)len) return false;

  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Init camera...");
  if (!initCamera()) {
    Serial.println("Camera init failed, halting.");
    while (true) delay(1000);
  }

  // ---------- Start SoftAP ----------
  Serial.println("Starting SoftAP...");
  WiFi.mode(WIFI_AP);
  bool apOk = WiFi.softAP(AP_SSID, AP_PASS);
  if (!apOk) {
    Serial.println("SoftAP start failed.");
  } else {
    Serial.print("SoftAP started. SSID: ");
    Serial.println(AP_SSID);
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());  // Typically 192.168.4.1[web:66]
  }

  server.begin();
  server.setNoDelay(true);
  Serial.printf("TCP server on port %u\n", TCP_PORT);
}

void loop() {
  // Accept client anytime (power order independent)
  if (!client || !client.connected()) {
    client.stop();
    client = server.available();
    if (client) {
      client.setNoDelay(true);
      Serial.println("Client connected.");
    } else {
      delay(10);
      return;
    }
  }

  // Capture frame
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    delay(10);
    return;
  }

  // Send JPEG frame
  bool ok = sendFrame(client, fb->buf, fb->len);
  esp_camera_fb_return(fb);

  if (!ok) {
    Serial.println("Client disconnected during send.");
    client.stop();
    return;
  }

  // Optional throttle if needed
  // delay(1);
}
