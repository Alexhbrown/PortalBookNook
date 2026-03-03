#include "esp_camera.h"
#include <WiFi.h>
#include <HTTPClient.h>

// =========================
// Wi-Fi
// =========================
const char* WIFI_SSID = "ahb";
const char* WIFI_PASS = "0987654321";

// Mac/PC running the receiver script
const char* UPLOAD_URL = "http://192.168.7.205:8080/upload";

// Capture interval
const uint32_t CAPTURE_INTERVAL_MS = 3000;

// =========================
// ESP32-CAM AI Thinker pinout (adjust if your board differs)
// =========================
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

bool initCamera() {
  camera_config_t config = {};   // IMPORTANT: zero-init

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

  // For debugging image quality, bump this up from QVGA
  config.frame_size   = FRAMESIZE_VGA;   // 640x480 (try SVGA if stable)
  config.jpeg_quality = 10;              // lower = better
  config.fb_count     = psramFound() ? 2 : 1;

#if defined(CAMERA_GRAB_LATEST)
  config.grab_mode    = CAMERA_GRAB_LATEST;
#endif
#if defined(CAMERA_FB_IN_PSRAM)
  config.fb_location  = CAMERA_FB_IN_PSRAM;
#endif

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }

  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    // OV3660 tuning (and generally helpful for washed-out output)
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_gain_ctrl(s, 1);
    s->set_exposure_ctrl(s, 1);

    s->set_brightness(s, 0);   // -2..2
    s->set_contrast(s, 2);     // -2..2
    s->set_saturation(s, 1);   // -2..2
    // Help with harsh scenes
    s->set_aec2(s, 1);         // improved auto exposure (if supported)
    s->set_ae_level(s, -1);    // darker overall to protect highlights (-2..2)

    // Optional if supported by your core/sensor:
    s->set_gainceiling(s, (gainceiling_t)2);  // lower gain = less noisy
    // Optional: flip/mirror if image orientation is weird
    // s->set_vflip(s, 1);
    // s->set_hmirror(s, 1);
  }

  Serial.printf("PSRAM: %s\n", psramFound() ? "YES" : "NO");
  return true;
}

bool uploadFrame(camera_fb_t* fb) {
  if (!fb || fb->format != PIXFORMAT_JPEG) {
    Serial.println("No JPEG frame");
    return false;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected");
    return false;
  }

  HTTPClient http;
  WiFiClient client;

  http.begin(client, UPLOAD_URL);
  http.addHeader("Content-Type", "image/jpeg");
  http.addHeader("X-Device", "esp32-cam");
  http.addHeader("X-Length", String(fb->len));

  int code = http.POST(fb->buf, fb->len);
  Serial.printf("POST -> %d, bytes=%u\n", code, (unsigned)fb->len);

  if (code > 0) {
    String resp = http.getString();
    Serial.println(resp);
  } else {
    Serial.printf("HTTP error: %s\n", http.errorToString(code).c_str());
  }

  http.end();
  return code > 0 && code < 300;
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("Connecting to WiFi: %s", WIFI_SSID);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected, IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi connect failed");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\nBooting ESP32-CAM uploader...");

  connectWiFi();

  if (!initCamera()) {
    Serial.println("Camera init failed, halting.");
    while (true) delay(1000);
  }
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
    delay(1000);
    return;
  }

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Capture failed");
    delay(1000);
    return;
  }

  uploadFrame(fb);
  esp_camera_fb_return(fb);

  delay(CAPTURE_INTERVAL_MS);
}