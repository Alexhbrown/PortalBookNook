// ESP32-CAM: SoftAP + JPEG over TCP server

#include "esp_camera.h"
#include <WiFi.h>

// ---------- Wi-Fi SoftAP config ----------
const char* AP_SSID = "BluePortal";
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
  camera_config_t config = {};              // <-- IMPORTANT: zero-init

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

  // Keep it QVGA for your 320x240 screen
  config.frame_size   = FRAMESIZE_QVGA;

  // LOWER = better quality (try 10–12)
  config.jpeg_quality = 10;

  // 2 buffers is fine if PSRAM exists; if you get instability try 1
  config.fb_count     = 2;

  // These fields exist in many cores; harmless if yours ignores them
  config.grab_mode    = CAMERA_GRAB_LATEST;
  config.fb_location  = CAMERA_FB_IN_PSRAM;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }

  sensor_t* s = esp_camera_sensor_get();

 
  // Core image tuning (from your UI)
  s->set_brightness(s, 0);      // Brightness slider centered
  s->set_contrast(s, 1);        // Contrast slightly up
  s->set_saturation(s, 2);      // Saturation up
  s->set_sharpness(s, 1);       // Sharpness up (if supported)
  s->set_denoise(s, 0);         // De-noise = Auto/low (0 is a good match)

  // Exposure / gain behavior
  s->set_aec2(s, 1);            // "Night mode"/advanced exposure style ON in many builds
  s->set_ae_level(s, 0);        // Exposure Level centered
  s->set_aec_value(s, 300);     // Only used if AEC is OFF; harmless otherwise
  s->set_exposure_ctrl(s, 1);   // AEC Enable ON
  s->set_gain_ctrl(s, 1);       // AGC ON
  s->set_gainceiling(s, GAINCEILING_2X); // low gain ceiling (UI looked near low end)

  // White balance
  s->set_whitebal(s, 1);        // AWB Enable ON
  s->set_awb_gain(s, 1);        // Advanced AWB ON
  // Manual AWB OFF (no direct setting needed if AWB is on)

  // Image processing toggles
  s->set_raw_gma(s, 1);         // GMA Enable ON
  s->set_lenc(s, 1);            // Lens Correction ON
  s->set_hmirror(s, 0);         // H-Mirror OFF
  s->set_vflip(s, 1);           // V-Flip ON
  s->set_bpc(s, 1);             // BPC ON
  s->set_wpc(s, 1);             // WPC ON
  s->set_colorbar(s, 0);        // Color bar OFF

  // Optional: no special effect
  s->set_special_effect(s, 0);  // 0 = No Effect

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
