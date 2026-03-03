// ESP32-CAM (AI-Thinker):
// - SoftAP "OrangePortal"
// - TCP JPEG stream for CYD on port 5000
// - HTTP tuning UI on port 80 (esp_http_server)
// - Persistent camera settings via Preferences (NVS)

#include "esp_camera.h"
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <WiFi.h>
#include <esp_http_server.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <stdlib.h>
#include <string.h>

// ---------- Wi-Fi SoftAP config ----------
const char* AP_SSID = "OrangePortal";
const char* AP_PASS = "TheCakeIsALie";
const uint16_t TCP_PORT = 5000;

const char* OTA_WIFI_SSID = "ahb-IOT";
const char* OTA_WIFI_PASS = "0987654321";
const char* OTA_HOSTNAME = "portal-bluecam";
const uint32_t OTA_MODE_TIMEOUT_MS = 10UL * 60UL * 1000UL;

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

struct CameraSettings {
  int quality;
  int brightness;
  int contrast;
  int saturation;
  int sharpness;
  int denoise;
  int aec2;
  int ae_level;
  int aec_value;
  int aec;
  int agc;
  int agc_gain;
  int gainceiling;
  int awb;
  int awb_gain;
  int wb_mode;
  int lenc;
  int hmirror;
  int vflip;
  int bpc;
  int wpc;
  int raw_gma;
  int special_effect;
  int colorbar;
};

Preferences prefs;
WiFiServer tcpServer(TCP_PORT);
WiFiClient tcpClient;
httpd_handle_t httpServer = nullptr;

SemaphoreHandle_t camMutex = nullptr;
SemaphoreHandle_t frameMutex = nullptr;

CameraSettings settings;

static uint8_t* latestFrameBuf = nullptr;
static size_t latestFrameLen = 0;
static const size_t LATEST_FRAME_BUF_SIZE = 100 * 1024;

static uint16_t streamFrameIntervalMs = 80;   // ~12 fps max while CYD is active
static uint16_t previewFrameIntervalMs = 300; // preview cadence for web clients
static const uint32_t PREVIEW_ACTIVE_WINDOW_MS = 5000;

static uint32_t lastCaptureMs = 0;
static volatile uint32_t lastPreviewRequestMs = 0;

static bool lastTcpConnected = false;
static uint32_t captureHitCount = 0;

static bool otaMode = false;
static bool otaNetworkReady = false;
static bool otaHttpServerStarted = false;
static uint32_t otaModeStartMs = 0;
static uint32_t otaReconnectAttemptMs = 0;

static bool restartScheduled = false;
static uint32_t restartAtMs = 0;

int clampInt(int value, int minValue, int maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

void logHttp(const char* path) {
  Serial.printf("[HTTP] %s\n", path);
}

void logCaptureOccasional() {
  captureHitCount++;
  if ((captureHitCount % 20) == 1) {
    Serial.printf("[HTTP] /capture.jpg (count=%lu)\n", (unsigned long)captureHitCount);
  }
}

void scheduleRestart(uint32_t delayMs = 350) {
  restartScheduled = true;
  restartAtMs = millis() + delayMs;
}

bool restartDue() {
  if (!restartScheduled) return false;
  return (int32_t)(millis() - restartAtMs) >= 0;
}

CameraSettings defaultSettings() {
  CameraSettings s = {};
  s.quality = 10;
  s.brightness = 0;
  s.contrast = 1;
  s.saturation = 2;
  s.sharpness = 1;
  s.denoise = 0;
  s.aec2 = 1;
  s.ae_level = 0;
  s.aec_value = 300;
  s.aec = 1;
  s.agc = 1;
  s.agc_gain = 0;
  s.gainceiling = 0;
  s.awb = 1;
  s.awb_gain = 1;
  s.wb_mode = 0;
  s.lenc = 1;
  s.hmirror = 0;
  s.vflip = 1;
  s.bpc = 1;
  s.wpc = 1;
  s.raw_gma = 1;
  s.special_effect = 0;
  s.colorbar = 0;
  return s;
}

void clampAllSettings() {
  settings.quality = clampInt(settings.quality, 4, 63);
  settings.brightness = clampInt(settings.brightness, -2, 2);
  settings.contrast = clampInt(settings.contrast, -2, 2);
  settings.saturation = clampInt(settings.saturation, -2, 2);
  settings.sharpness = clampInt(settings.sharpness, -2, 3);
  settings.denoise = clampInt(settings.denoise, 0, 8);
  settings.aec2 = clampInt(settings.aec2, 0, 1);
  settings.ae_level = clampInt(settings.ae_level, -2, 2);
  settings.aec_value = clampInt(settings.aec_value, 0, 1200);
  settings.aec = clampInt(settings.aec, 0, 1);
  settings.agc = clampInt(settings.agc, 0, 1);
  settings.agc_gain = clampInt(settings.agc_gain, 0, 30);
  settings.gainceiling = clampInt(settings.gainceiling, 0, 6);
  settings.awb = clampInt(settings.awb, 0, 1);
  settings.awb_gain = clampInt(settings.awb_gain, 0, 1);
  settings.wb_mode = clampInt(settings.wb_mode, 0, 4);
  settings.lenc = clampInt(settings.lenc, 0, 1);
  settings.hmirror = clampInt(settings.hmirror, 0, 1);
  settings.vflip = clampInt(settings.vflip, 0, 1);
  settings.bpc = clampInt(settings.bpc, 0, 1);
  settings.wpc = clampInt(settings.wpc, 0, 1);
  settings.raw_gma = clampInt(settings.raw_gma, 0, 1);
  settings.special_effect = clampInt(settings.special_effect, 0, 6);
  settings.colorbar = clampInt(settings.colorbar, 0, 1);
}

void loadSettingsFromNvs() {
  settings = defaultSettings();
  prefs.begin("portalbcam", false);

  settings.quality = prefs.getInt("quality", settings.quality);
  settings.brightness = prefs.getInt("bright", settings.brightness);
  settings.contrast = prefs.getInt("contrast", settings.contrast);
  settings.saturation = prefs.getInt("satur", settings.saturation);
  settings.sharpness = prefs.getInt("sharp", settings.sharpness);
  settings.denoise = prefs.getInt("denoise", settings.denoise);
  settings.aec2 = prefs.getInt("aec2", settings.aec2);
  settings.ae_level = prefs.getInt("aelevel", settings.ae_level);
  settings.aec_value = prefs.getInt("aecvalue", settings.aec_value);
  settings.aec = prefs.getInt("aec", settings.aec);
  settings.agc = prefs.getInt("agc", settings.agc);
  settings.agc_gain = prefs.getInt("agcgain", settings.agc_gain);
  settings.gainceiling = prefs.getInt("gainceil", settings.gainceiling);
  settings.awb = prefs.getInt("awb", settings.awb);
  settings.awb_gain = prefs.getInt("awbgain", settings.awb_gain);
  settings.wb_mode = prefs.getInt("wbmode", settings.wb_mode);
  settings.lenc = prefs.getInt("lenc", settings.lenc);
  settings.hmirror = prefs.getInt("hmirror", settings.hmirror);
  settings.vflip = prefs.getInt("vflip", settings.vflip);
  settings.bpc = prefs.getInt("bpc", settings.bpc);
  settings.wpc = prefs.getInt("wpc", settings.wpc);
  settings.raw_gma = prefs.getInt("rawgma", settings.raw_gma);
  settings.special_effect = prefs.getInt("effect", settings.special_effect);
  settings.colorbar = prefs.getInt("colorbar", settings.colorbar);
  streamFrameIntervalMs = (uint16_t)clampInt(prefs.getInt("streamms", streamFrameIntervalMs), 30, 500);
  previewFrameIntervalMs = (uint16_t)clampInt(prefs.getInt("previewms", previewFrameIntervalMs), 100, 2000);

  clampAllSettings();
}

void saveAllSettingsToNvs() {
  prefs.putInt("quality", settings.quality);
  prefs.putInt("bright", settings.brightness);
  prefs.putInt("contrast", settings.contrast);
  prefs.putInt("satur", settings.saturation);
  prefs.putInt("sharp", settings.sharpness);
  prefs.putInt("denoise", settings.denoise);
  prefs.putInt("aec2", settings.aec2);
  prefs.putInt("aelevel", settings.ae_level);
  prefs.putInt("aecvalue", settings.aec_value);
  prefs.putInt("aec", settings.aec);
  prefs.putInt("agc", settings.agc);
  prefs.putInt("agcgain", settings.agc_gain);
  prefs.putInt("gainceil", settings.gainceiling);
  prefs.putInt("awb", settings.awb);
  prefs.putInt("awbgain", settings.awb_gain);
  prefs.putInt("wbmode", settings.wb_mode);
  prefs.putInt("lenc", settings.lenc);
  prefs.putInt("hmirror", settings.hmirror);
  prefs.putInt("vflip", settings.vflip);
  prefs.putInt("bpc", settings.bpc);
  prefs.putInt("wpc", settings.wpc);
  prefs.putInt("rawgma", settings.raw_gma);
  prefs.putInt("effect", settings.special_effect);
  prefs.putInt("colorbar", settings.colorbar);
  prefs.putInt("streamms", streamFrameIntervalMs);
  prefs.putInt("previewms", previewFrameIntervalMs);
}

bool applyAllSensorSettings() {
  if (xSemaphoreTake(camMutex, pdMS_TO_TICKS(300)) != pdTRUE) return false;

  sensor_t* s = esp_camera_sensor_get();
  if (!s) {
    xSemaphoreGive(camMutex);
    return false;
  }

  s->set_framesize(s, FRAMESIZE_QVGA);
  s->set_quality(s, settings.quality);
  s->set_brightness(s, settings.brightness);
  s->set_contrast(s, settings.contrast);
  s->set_saturation(s, settings.saturation);
  s->set_sharpness(s, settings.sharpness);
  s->set_denoise(s, settings.denoise);

  s->set_aec2(s, settings.aec2);
  s->set_ae_level(s, settings.ae_level);
  s->set_aec_value(s, settings.aec_value);
  s->set_exposure_ctrl(s, settings.aec);
  s->set_gain_ctrl(s, settings.agc);
  s->set_agc_gain(s, settings.agc_gain);
  s->set_gainceiling(s, (gainceiling_t)settings.gainceiling);

  s->set_whitebal(s, settings.awb);
  s->set_awb_gain(s, settings.awb_gain);
  s->set_wb_mode(s, settings.wb_mode);

  s->set_lenc(s, settings.lenc);
  s->set_hmirror(s, settings.hmirror);
  s->set_vflip(s, settings.vflip);
  s->set_bpc(s, settings.bpc);
  s->set_wpc(s, settings.wpc);
  s->set_raw_gma(s, settings.raw_gma);
  s->set_special_effect(s, settings.special_effect);
  s->set_colorbar(s, settings.colorbar);

  xSemaphoreGive(camMutex);
  return true;
}

int applySettingByName(const char* var, int value) {
  if (xSemaphoreTake(camMutex, pdMS_TO_TICKS(300)) != pdTRUE) return -1;

  sensor_t* s = esp_camera_sensor_get();
  if (!s) {
    xSemaphoreGive(camMutex);
    return -1;
  }

  int result = -1;

  if (strcmp(var, "quality") == 0) {
    value = clampInt(value, 4, 63);
    result = s->set_quality(s, value);
    if (result == 0) settings.quality = value;
  } else if (strcmp(var, "brightness") == 0) {
    value = clampInt(value, -2, 2);
    result = s->set_brightness(s, value);
    if (result == 0) settings.brightness = value;
  } else if (strcmp(var, "contrast") == 0) {
    value = clampInt(value, -2, 2);
    result = s->set_contrast(s, value);
    if (result == 0) settings.contrast = value;
  } else if (strcmp(var, "saturation") == 0) {
    value = clampInt(value, -2, 2);
    result = s->set_saturation(s, value);
    if (result == 0) settings.saturation = value;
  } else if (strcmp(var, "sharpness") == 0) {
    value = clampInt(value, -2, 3);
    result = s->set_sharpness(s, value);
    if (result == 0) settings.sharpness = value;
  } else if (strcmp(var, "denoise") == 0) {
    value = clampInt(value, 0, 8);
    result = s->set_denoise(s, value);
    if (result == 0) settings.denoise = value;
  } else if (strcmp(var, "aec2") == 0) {
    value = clampInt(value, 0, 1);
    result = s->set_aec2(s, value);
    if (result == 0) settings.aec2 = value;
  } else if (strcmp(var, "ae_level") == 0) {
    value = clampInt(value, -2, 2);
    result = s->set_ae_level(s, value);
    if (result == 0) settings.ae_level = value;
  } else if (strcmp(var, "aec_value") == 0) {
    value = clampInt(value, 0, 1200);
    result = s->set_aec_value(s, value);
    if (result == 0) settings.aec_value = value;
  } else if (strcmp(var, "aec") == 0) {
    value = clampInt(value, 0, 1);
    result = s->set_exposure_ctrl(s, value);
    if (result == 0) settings.aec = value;
  } else if (strcmp(var, "agc") == 0) {
    value = clampInt(value, 0, 1);
    result = s->set_gain_ctrl(s, value);
    if (result == 0) settings.agc = value;
  } else if (strcmp(var, "agc_gain") == 0) {
    value = clampInt(value, 0, 30);
    result = s->set_agc_gain(s, value);
    if (result == 0) settings.agc_gain = value;
  } else if (strcmp(var, "gainceiling") == 0) {
    value = clampInt(value, 0, 6);
    result = s->set_gainceiling(s, (gainceiling_t)value);
    if (result == 0) settings.gainceiling = value;
  } else if (strcmp(var, "awb") == 0) {
    value = clampInt(value, 0, 1);
    result = s->set_whitebal(s, value);
    if (result == 0) settings.awb = value;
  } else if (strcmp(var, "awb_gain") == 0) {
    value = clampInt(value, 0, 1);
    result = s->set_awb_gain(s, value);
    if (result == 0) settings.awb_gain = value;
  } else if (strcmp(var, "wb_mode") == 0) {
    value = clampInt(value, 0, 4);
    result = s->set_wb_mode(s, value);
    if (result == 0) settings.wb_mode = value;
  } else if (strcmp(var, "lenc") == 0) {
    value = clampInt(value, 0, 1);
    result = s->set_lenc(s, value);
    if (result == 0) settings.lenc = value;
  } else if (strcmp(var, "hmirror") == 0) {
    value = clampInt(value, 0, 1);
    result = s->set_hmirror(s, value);
    if (result == 0) settings.hmirror = value;
  } else if (strcmp(var, "vflip") == 0) {
    value = clampInt(value, 0, 1);
    result = s->set_vflip(s, value);
    if (result == 0) settings.vflip = value;
  } else if (strcmp(var, "bpc") == 0) {
    value = clampInt(value, 0, 1);
    result = s->set_bpc(s, value);
    if (result == 0) settings.bpc = value;
  } else if (strcmp(var, "wpc") == 0) {
    value = clampInt(value, 0, 1);
    result = s->set_wpc(s, value);
    if (result == 0) settings.wpc = value;
  } else if (strcmp(var, "raw_gma") == 0) {
    value = clampInt(value, 0, 1);
    result = s->set_raw_gma(s, value);
    if (result == 0) settings.raw_gma = value;
  } else if (strcmp(var, "special_effect") == 0) {
    value = clampInt(value, 0, 6);
    result = s->set_special_effect(s, value);
    if (result == 0) settings.special_effect = value;
  } else if (strcmp(var, "colorbar") == 0) {
    value = clampInt(value, 0, 1);
    result = s->set_colorbar(s, value);
    if (result == 0) settings.colorbar = value;
  }

  xSemaphoreGive(camMutex);

  if (result == 0) {
    saveAllSettingsToNvs();
  }

  return result;
}

bool initCamera() {
  camera_config_t config = {};

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;

  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;

  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;

  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;

  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = settings.quality;
  config.fb_count = psramFound() ? 2 : 1;

#if defined(CAMERA_GRAB_LATEST)
  config.grab_mode = CAMERA_GRAB_LATEST;
#endif
#if defined(CAMERA_FB_IN_PSRAM)
  config.fb_location = CAMERA_FB_IN_PSRAM;
#endif

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }

  return applyAllSensorSettings();
}

bool captureLatestFrame() {
  if (xSemaphoreTake(camMutex, pdMS_TO_TICKS(300)) != pdTRUE) return false;

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    xSemaphoreGive(camMutex);
    return false;
  }

  bool ok = false;
  if (fb->len <= LATEST_FRAME_BUF_SIZE && xSemaphoreTake(frameMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
    memcpy(latestFrameBuf, fb->buf, fb->len);
    latestFrameLen = fb->len;
    xSemaphoreGive(frameMutex);
    ok = true;
  }

  esp_camera_fb_return(fb);
  xSemaphoreGive(camMutex);

  return ok;
}

bool sendTcpFrameFromCache(WiFiClient& c) {
  if (!c.connected()) return false;

  if (xSemaphoreTake(frameMutex, pdMS_TO_TICKS(200)) != pdTRUE) return false;

  size_t len = latestFrameLen;
  if (len == 0) {
    xSemaphoreGive(frameMutex);
    return false;
  }

  uint8_t header[4];
  header[0] = (len >> 24) & 0xFF;
  header[1] = (len >> 16) & 0xFF;
  header[2] = (len >> 8) & 0xFF;
  header[3] = len & 0xFF;

  if (c.write(header, 4) != 4) {
    xSemaphoreGive(frameMutex);
    return false;
  }

  size_t sent = 0;
  while (sent < len) {
    size_t chunk = len - sent;
    if (chunk > 1024) chunk = 1024;
    int w = c.write(latestFrameBuf + sent, chunk);
    if (w <= 0) {
      xSemaphoreGive(frameMutex);
      return false;
    }
    sent += (size_t)w;
    delay(0);
  }

  xSemaphoreGive(frameMutex);
  return true;
}

void setOtaModeFlag(bool enabled) {
  prefs.putBool("otamode", enabled);
}

bool connectOtaStaBlocking(uint32_t timeoutMs) {
  if (WiFi.status() == WL_CONNECTED) return true;

  WiFi.disconnect(true, true);
  delay(80);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(OTA_WIFI_SSID, OTA_WIFI_PASS);

  Serial.printf("[OTA] Connecting to %s", OTA_WIFI_SSID);
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeoutMs) {
    Serial.print(".");
    delay(250);
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[OTA] Wi-Fi connect timed out.");
    return false;
  }

  Serial.printf("[OTA] Wi-Fi connected, IP: %s\n", WiFi.localIP().toString().c_str());
  return true;
}

void beginArduinoOtaIfReady() {
  if (otaNetworkReady || WiFi.status() != WL_CONNECTED) return;

  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.onStart([]() { Serial.println("[OTA] Update start"); });
  ArduinoOTA.onEnd([]() { Serial.println("\n[OTA] Update end"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    static uint32_t lastPrint = 0;
    uint32_t now = millis();
    if (now - lastPrint < 500) return;
    lastPrint = now;
    unsigned int pct = (total == 0) ? 0 : (progress * 100U) / total;
    Serial.printf("[OTA] Progress: %u%%\n", pct);
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("[OTA] Error[%u]\n", (unsigned)error);
  });
  ArduinoOTA.begin();
  otaNetworkReady = true;
  Serial.printf("[OTA] Ready. Hostname: %s, IP: %s, Port: 3232\n",
                OTA_HOSTNAME, WiFi.localIP().toString().c_str());
}

static const char OTA_INDEX_HTML[] = R"HTML(
<!doctype html>
<html>
<head>
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Portal BlueCam OTA Mode</title>
  <style>
    body { font-family: -apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif; margin: 14px; background:#111; color:#f2f2f2; }
    h2 { margin: 0 0 8px 0; }
    .row { margin: 8px 0; padding: 10px; background:#1b1b1b; border-radius:10px; }
    .line button { padding:8px 12px; border:0; border-radius:8px; background:#2b6fff; color:#fff; }
    .small { color:#aaa; font-size:13px; margin:6px 0; }
  </style>
</head>
<body>
  <h2>Portal BlueCam OTA Mode</h2>
  <div class="small">Device is on ahb-IOT for Arduino OTA uploads.</div>
  <div id="statusLine" class="small">Loading...</div>
  <div class="row line">
    <button onclick="refreshStatus()">Refresh</button>
    <button onclick="exitOta()">Exit OTA Mode</button>
  </div>
  <script>
    async function refreshStatus() {
      try {
        const r = await fetch('/status');
        const s = await r.json();
        document.getElementById('statusLine').textContent = `IP ${s.ip} | host ${s.hostname} | timeout ${s.ota_timeout_sec}s`;
      } catch (_) {
        document.getElementById('statusLine').textContent = 'Status fetch failed';
      }
    }
    async function exitOta() {
      const r = await fetch('/ota/exit');
      alert(await r.text());
    }
    refreshStatus();
    setInterval(refreshStatus, 2000);
  </script>
</body>
</html>
)HTML";

esp_err_t sendStatusJson(httpd_req_t* req) {
  char json[760];
  uint32_t remainingSec = 0;
  if (otaMode) {
    uint32_t elapsed = millis() - otaModeStartMs;
    if (elapsed < OTA_MODE_TIMEOUT_MS) {
      remainingSec = (OTA_MODE_TIMEOUT_MS - elapsed) / 1000U;
    }
  }
  int n = snprintf(
      json, sizeof(json),
      "{\"quality\":%d,\"brightness\":%d,\"contrast\":%d,\"saturation\":%d,"
      "\"sharpness\":%d,\"denoise\":%d,\"aec2\":%d,\"ae_level\":%d,"
      "\"aec_value\":%d,\"aec\":%d,\"agc\":%d,\"agc_gain\":%d,"
      "\"gainceiling\":%d,\"awb\":%d,\"awb_gain\":%d,\"wb_mode\":%d,"
      "\"lenc\":%d,\"hmirror\":%d,\"vflip\":%d,\"bpc\":%d,\"wpc\":%d,"
      "\"raw_gma\":%d,\"special_effect\":%d,\"colorbar\":%d,"
      "\"stream_interval_ms\":%u,\"preview_interval_ms\":%u,"
      "\"ota_mode\":%d,\"ota_timeout_sec\":%lu,\"ip\":\"%s\",\"hostname\":\"%s\"}",
      settings.quality, settings.brightness, settings.contrast, settings.saturation,
      settings.sharpness, settings.denoise, settings.aec2, settings.ae_level,
      settings.aec_value, settings.aec, settings.agc, settings.agc_gain,
      settings.gainceiling, settings.awb, settings.awb_gain, settings.wb_mode,
      settings.lenc, settings.hmirror, settings.vflip, settings.bpc, settings.wpc,
      settings.raw_gma, settings.special_effect, settings.colorbar,
      (unsigned)streamFrameIntervalMs, (unsigned)previewFrameIntervalMs,
      otaMode ? 1 : 0, (unsigned long)remainingSec,
      WiFi.localIP().toString().c_str(), OTA_HOSTNAME);

  if (n <= 0) return ESP_FAIL;

  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

static const char INDEX_HTML[] = R"HTML(
<!doctype html>
<html>
<head>
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>OrangePortal Camera Tuning</title>
  <style>
    body { font-family: -apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif; margin: 14px; background:#111; color:#f2f2f2; }
    h2 { margin: 0 0 8px 0; }
    .row { margin: 8px 0; padding: 8px; background:#1b1b1b; border-radius:10px; }
    .name { display:block; font-weight:600; margin-bottom:4px; }
    .line { display:flex; align-items:center; gap:8px; }
    .line input[type=range] { flex:1; }
    .line button { padding:8px 12px; border:0; border-radius:8px; background:#2b6fff; color:#fff; }
    img { width:100%; max-width:480px; border-radius:10px; background:#000; }
    .small { color:#aaa; font-size:13px; margin:6px 0; }
  </style>
</head>
<body>
  <h2>OrangePortal Camera Tuning</h2>
  <div class="small">Connect iPhone to OrangePortal. Settings auto-save on change.</div>
  <img id="preview" src="" alt="Camera preview">
  <div class="row line">
    <label><input id="autoRefresh" type="checkbox"> Live preview</label>
    <button onclick="refreshOnce()">Refresh Now</button>
    <button onclick="resetDefaults()">Reset Defaults</button>
  </div>
  <div class="row line">
    <button onclick="enterOtaMode()">Enter OTA Mode (ahb-IOT)</button>
  </div>

  <div class="row"><span class="name">JPEG Quality (lower = better)</span><div class="line"><input id="quality" type="range" min="4" max="63"><span id="quality_v"></span></div></div>
  <div class="row"><span class="name">Brightness</span><div class="line"><input id="brightness" type="range" min="-2" max="2"><span id="brightness_v"></span></div></div>
  <div class="row"><span class="name">Contrast</span><div class="line"><input id="contrast" type="range" min="-2" max="2"><span id="contrast_v"></span></div></div>
  <div class="row"><span class="name">Saturation</span><div class="line"><input id="saturation" type="range" min="-2" max="2"><span id="saturation_v"></span></div></div>
  <div class="row"><span class="name">Sharpness</span><div class="line"><input id="sharpness" type="range" min="-2" max="3"><span id="sharpness_v"></span></div></div>
  <div class="row"><span class="name">Denoise</span><div class="line"><input id="denoise" type="range" min="0" max="8"><span id="denoise_v"></span></div></div>
  <div class="row"><span class="name">AE Level</span><div class="line"><input id="ae_level" type="range" min="-2" max="2"><span id="ae_level_v"></span></div></div>
  <div class="row"><span class="name">AEC Value</span><div class="line"><input id="aec_value" type="range" min="0" max="1200"><span id="aec_value_v"></span></div></div>
  <div class="row"><span class="name">AGC Gain</span><div class="line"><input id="agc_gain" type="range" min="0" max="30"><span id="agc_gain_v"></span></div></div>
  <div class="row"><span class="name">Gain Ceiling</span><div class="line"><input id="gainceiling" type="range" min="0" max="6"><span id="gainceiling_v"></span></div></div>
  <div class="row"><span class="name">WB Mode</span><div class="line"><input id="wb_mode" type="range" min="0" max="4"><span id="wb_mode_v"></span></div></div>
  <div class="row"><span class="name">Special Effect</span><div class="line"><input id="special_effect" type="range" min="0" max="6"><span id="special_effect_v"></span></div></div>
  <div class="row"><span class="name">Stream Interval ms (lower = higher FPS)</span><div class="line"><input id="stream_interval_ms" type="range" min="30" max="500"><span id="stream_interval_ms_v"></span></div></div>
  <div class="row"><span class="name">Preview Interval ms</span><div class="line"><input id="preview_interval_ms" type="range" min="100" max="2000"><span id="preview_interval_ms_v"></span></div></div>

  <div class="row line"><label><input id="aec2" type="checkbox"> AEC2</label><label><input id="aec" type="checkbox"> AEC</label><label><input id="agc" type="checkbox"> AGC</label><label><input id="awb" type="checkbox"> AWB</label></div>
  <div class="row line"><label><input id="awb_gain" type="checkbox"> AWB Gain</label><label><input id="lenc" type="checkbox"> Lens Corr</label><label><input id="hmirror" type="checkbox"> H Mirror</label><label><input id="vflip" type="checkbox"> V Flip</label></div>
  <div class="row line"><label><input id="bpc" type="checkbox"> BPC</label><label><input id="wpc" type="checkbox"> WPC</label><label><input id="raw_gma" type="checkbox"> Raw GMA</label><label><input id="colorbar" type="checkbox"> Colorbar</label></div>

  <script>
    const sliderIds = ["quality","brightness","contrast","saturation","sharpness","denoise","ae_level","aec_value","agc_gain","gainceiling","wb_mode","special_effect","stream_interval_ms","preview_interval_ms"];
    const checkIds  = ["aec2","aec","agc","awb","awb_gain","lenc","hmirror","vflip","bpc","wpc","raw_gma","colorbar"];

    const preview = document.getElementById("preview");
    const autoRefresh = document.getElementById("autoRefresh");
    let previewBusy = false;
    let previewLoopTimer = null;
    let previewBusyTimeout = null;

    async function sendControl(id, val) {
      try { await fetch(`/control?var=${id}&val=${val}`); } catch (_) {}
    }

    function refreshOnce() {
      if (previewBusy) return;
      previewBusy = true;
      if (previewBusyTimeout) clearTimeout(previewBusyTimeout);
      preview.onload = () => { previewBusy = false; };
      preview.onerror = () => { previewBusy = false; };
      previewBusyTimeout = setTimeout(() => { previewBusy = false; }, 4000);
      preview.src = "/capture.jpg?t=" + Date.now();
    }

    function runPreviewLoop() {
      if (previewLoopTimer) clearTimeout(previewLoopTimer);
      if (!autoRefresh.checked) return;
      previewLoopTimer = setTimeout(() => {
        refreshOnce();
        runPreviewLoop();
      }, 1200);
    }

    async function resetDefaults() {
      await fetch("/reset");
      await loadStatus();
      refreshOnce();
    }

    async function enterOtaMode() {
      if (!confirm("Reboot camera into OTA mode on ahb-IOT?")) return;
      try {
        const r = await fetch("/ota/start");
        alert(await r.text());
      } catch (_) {
        alert("Failed to request OTA mode");
      }
    }

    async function loadStatus() {
      try {
        const r = await fetch("/status");
        const s = await r.json();

        sliderIds.forEach(id => {
          const el = document.getElementById(id);
          const out = document.getElementById(id + "_v");
          if (!el || !(id in s)) return;
          el.value = s[id];
          if (out) out.textContent = s[id];
        });

        checkIds.forEach(id => {
          const el = document.getElementById(id);
          if (!el || !(id in s)) return;
          el.checked = !!s[id];
        });
      } catch (_) {}
    }

    sliderIds.forEach(id => {
      const el = document.getElementById(id);
      const out = document.getElementById(id + "_v");
      if (!el) return;
      el.addEventListener("input", () => { if (out) out.textContent = el.value; });
      el.addEventListener("change", () => sendControl(id, el.value));
    });

    checkIds.forEach(id => {
      const el = document.getElementById(id);
      if (!el) return;
      el.addEventListener("change", () => sendControl(id, el.checked ? 1 : 0));
    });

    autoRefresh.addEventListener("change", () => {
      if (autoRefresh.checked) refreshOnce();
      runPreviewLoop();
    });

    loadStatus().then(refreshOnce);
  </script>
</body>
</html>
)HTML";

esp_err_t handleIndex(httpd_req_t* req) {
  logHttp("/");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

esp_err_t handlePing(httpd_req_t* req) {
  logHttp("/ping");
  char msg[96];
  snprintf(msg, sizeof(msg), "ok uptime_ms=%lu heap=%u", (unsigned long)millis(), (unsigned)ESP.getFreeHeap());
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, msg, HTTPD_RESP_USE_STRLEN);
}

esp_err_t handleStatus(httpd_req_t* req) {
  logHttp("/status");
  return sendStatusJson(req);
}

esp_err_t handleCapture(httpd_req_t* req) {
  logCaptureOccasional();
  lastPreviewRequestMs = millis();

  size_t lenSnapshot = 0;
  if (xSemaphoreTake(frameMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    lenSnapshot = latestFrameLen;
    xSemaphoreGive(frameMutex);
  }

  if (lenSnapshot == 0) {
    captureLatestFrame();
  }

  if (xSemaphoreTake(frameMutex, pdMS_TO_TICKS(300)) != pdTRUE) {
    httpd_resp_set_status(req, "503 Service Unavailable");
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "Frame mutex timeout", HTTPD_RESP_USE_STRLEN);
    return ESP_FAIL;
  }

  if (latestFrameLen == 0) {
    xSemaphoreGive(frameMutex);
    httpd_resp_set_status(req, "503 Service Unavailable");
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "No frame yet", HTTPD_RESP_USE_STRLEN);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  esp_err_t res = httpd_resp_send(req, (const char*)latestFrameBuf, latestFrameLen);

  xSemaphoreGive(frameMutex);
  return res;
}

esp_err_t handleControl(httpd_req_t* req) {
  logHttp("/control");

  char query[128] = {0};
  char var[32] = {0};
  char valStr[16] = {0};

  size_t queryLen = httpd_req_get_url_query_len(req);
  if (queryLen == 0 || queryLen >= sizeof(query)) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing query");
    return ESP_FAIL;
  }

  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad query");
    return ESP_FAIL;
  }

  if (httpd_query_key_value(query, "var", var, sizeof(var)) != ESP_OK ||
      httpd_query_key_value(query, "val", valStr, sizeof(valStr)) != ESP_OK) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing var/val");
    return ESP_FAIL;
  }

  int val = atoi(valStr);
  if (strcmp(var, "stream_interval_ms") == 0) {
    streamFrameIntervalMs = (uint16_t)clampInt(val, 30, 500);
    saveAllSettingsToNvs();
    return sendStatusJson(req);
  }
  if (strcmp(var, "preview_interval_ms") == 0) {
    previewFrameIntervalMs = (uint16_t)clampInt(val, 100, 2000);
    saveAllSettingsToNvs();
    return sendStatusJson(req);
  }

  int result = applySettingByName(var, val);
  if (result != 0) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad or unsupported setting");
    return ESP_FAIL;
  }

  return sendStatusJson(req);
}

esp_err_t handleReset(httpd_req_t* req) {
  logHttp("/reset");
  settings = defaultSettings();
  streamFrameIntervalMs = 80;
  previewFrameIntervalMs = 300;
  clampAllSettings();
  applyAllSensorSettings();
  saveAllSettingsToNvs();
  return sendStatusJson(req);
}

esp_err_t handleOtaStart(httpd_req_t* req) {
  logHttp("/ota/start");
  setOtaModeFlag(true);
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_send(req, "Rebooting into OTA mode on ahb-IOT...", HTTPD_RESP_USE_STRLEN);
  scheduleRestart();
  return ESP_OK;
}

esp_err_t handleOtaExit(httpd_req_t* req) {
  logHttp("/ota/exit");
  setOtaModeFlag(false);
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_send(req, "Exiting OTA mode and rebooting...", HTTPD_RESP_USE_STRLEN);
  scheduleRestart();
  return ESP_OK;
}

esp_err_t handleOtaIndex(httpd_req_t* req) {
  logHttp("/");
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, OTA_INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

esp_err_t handleNoContent(httpd_req_t* req) {
  return httpd_resp_send(req, "", 0);
}

void registerHttpUri(const char* uri, esp_err_t (*handler)(httpd_req_t*)) {
  httpd_uri_t route = {};
  route.uri = uri;
  route.method = HTTP_GET;
  route.handler = handler;
  route.user_ctx = nullptr;
  httpd_register_uri_handler(httpServer, &route);
}

bool startHttpServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.max_uri_handlers = 20;
  config.stack_size = 8192;
  config.recv_wait_timeout = 10;
  config.send_wait_timeout = 10;

  if (httpd_start(&httpServer, &config) != ESP_OK) {
    Serial.println("Failed to start HTTP server");
    return false;
  }

  registerHttpUri("/", handleIndex);
  registerHttpUri("/ping", handlePing);
  registerHttpUri("/status", handleStatus);
  registerHttpUri("/capture.jpg", handleCapture);
  registerHttpUri("/control", handleControl);
  registerHttpUri("/reset", handleReset);
  registerHttpUri("/ota/start", handleOtaStart);
  registerHttpUri("/ota/exit", handleOtaExit);

  // iPhone / captive portal probes to avoid noisy 404 spam.
  registerHttpUri("/favicon.ico", handleNoContent);
  registerHttpUri("/apple-touch-icon.png", handleNoContent);
  registerHttpUri("/apple-touch-icon-precomposed.png", handleNoContent);
  registerHttpUri("/hotspot-detect.html", handleNoContent);
  registerHttpUri("/generate_204", handleNoContent);
  registerHttpUri("/connecttest.txt", handleNoContent);
  registerHttpUri("/ncsi.txt", handleNoContent);

  Serial.println("HTTP tuning UI started on port 80");
  return true;
}

bool startOtaHttpServer() {
  if (httpServer != nullptr) return true;

  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.max_uri_handlers = 12;
  config.stack_size = 8192;
  config.recv_wait_timeout = 10;
  config.send_wait_timeout = 10;

  if (httpd_start(&httpServer, &config) != ESP_OK) {
    Serial.println("[OTA] Failed to start OTA HTTP server");
    return false;
  }

  registerHttpUri("/", handleOtaIndex);
  registerHttpUri("/ping", handlePing);
  registerHttpUri("/status", sendStatusJson);
  registerHttpUri("/ota/exit", handleOtaExit);

  registerHttpUri("/favicon.ico", handleNoContent);
  registerHttpUri("/apple-touch-icon.png", handleNoContent);
  registerHttpUri("/apple-touch-icon-precomposed.png", handleNoContent);
  registerHttpUri("/hotspot-detect.html", handleNoContent);
  registerHttpUri("/generate_204", handleNoContent);
  registerHttpUri("/connecttest.txt", handleNoContent);
  registerHttpUri("/ncsi.txt", handleNoContent);

  otaHttpServerStarted = true;
  Serial.println("[OTA] HTTP status page started on port 80");
  return true;
}

void setupOtaMode() {
  otaMode = true;
  otaModeStartMs = millis();
  otaReconnectAttemptMs = 0;
  otaNetworkReady = false;

  Serial.println("[OTA] OTA mode requested. Skipping AP stream mode.");
  if (connectOtaStaBlocking(20000)) {
    beginArduinoOtaIfReady();
    startOtaHttpServer();
  }
}

void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  if (event == ARDUINO_EVENT_WIFI_AP_STACONNECTED) {
    const uint8_t* mac = info.wifi_ap_staconnected.mac;
    Serial.printf("[AP] STA connected: %02X:%02X:%02X:%02X:%02X:%02X aid=%d\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                  info.wifi_ap_staconnected.aid);
  } else if (event == ARDUINO_EVENT_WIFI_AP_STADISCONNECTED) {
    const uint8_t* mac = info.wifi_ap_stadisconnected.mac;
    Serial.printf("[AP] STA disconnected: %02X:%02X:%02X:%02X:%02X:%02X aid=%d\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                  info.wifi_ap_stadisconnected.aid);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  camMutex = xSemaphoreCreateMutex();
  frameMutex = xSemaphoreCreateMutex();

  if (!camMutex || !frameMutex) {
    Serial.println("Failed to create mutexes");
    while (true) delay(1000);
  }

  latestFrameBuf = (uint8_t*)(psramFound() ? ps_malloc(LATEST_FRAME_BUF_SIZE) : malloc(LATEST_FRAME_BUF_SIZE));
  if (!latestFrameBuf) {
    Serial.println("Failed to allocate latest frame buffer");
    while (true) delay(1000);
  }

  loadSettingsFromNvs();
  otaMode = prefs.getBool("otamode", false);

  if (otaMode) {
    setupOtaMode();
    return;
  }

  Serial.println("Initializing camera...");
  if (!initCamera()) {
    Serial.println("Camera init failed, halting.");
    while (true) delay(1000);
  }

  WiFi.onEvent(onWiFiEvent);
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);

  Serial.println("Starting SoftAP...");
  if (!WiFi.softAP(AP_SSID, AP_PASS)) {
    Serial.println("SoftAP start failed, halting.");
    while (true) delay(1000);
  }

  Serial.print("SoftAP SSID: ");
  Serial.println(AP_SSID);
  Serial.print("SoftAP IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.printf("AP stations: %d\n", WiFi.softAPgetStationNum());

  if (!startHttpServer()) {
    while (true) delay(1000);
  }

  tcpServer.begin();
  tcpServer.setNoDelay(true);
  Serial.printf("TCP stream server on port %u\n", TCP_PORT);
}

void loop() {
  if (restartDue()) {
    ESP.restart();
  }

  if (otaMode) {
    uint32_t now = millis();
    if (WiFi.status() == WL_CONNECTED) {
      if (!otaNetworkReady) {
        beginArduinoOtaIfReady();
      }
      if (!otaHttpServerStarted) {
        startOtaHttpServer();
      }
      if (otaNetworkReady) {
        ArduinoOTA.handle();
      }
    } else if ((now - otaReconnectAttemptMs) > 5000) {
      otaReconnectAttemptMs = now;
      otaNetworkReady = false;
      Serial.printf("[OTA] Reconnecting to %s...\n", OTA_WIFI_SSID);
      WiFi.disconnect(false, false);
      WiFi.begin(OTA_WIFI_SSID, OTA_WIFI_PASS);
    }

    if ((now - otaModeStartMs) >= OTA_MODE_TIMEOUT_MS) {
      Serial.println("[OTA] Timeout reached, returning to OrangePortal mode.");
      setOtaModeFlag(false);
      scheduleRestart();
    }

    delay(2);
    return;
  }

  bool tcpReady = (tcpClient && tcpClient.connected());

  if (lastTcpConnected && !tcpReady) {
    Serial.println("[TCP] CYD stream client disconnected.");
  }

  if (!tcpClient || !tcpClient.connected()) {
    tcpClient.stop();
    tcpClient = tcpServer.available();
    if (tcpClient) {
      tcpClient.setNoDelay(true);
      Serial.printf("[TCP] CYD stream client connected from %s\n", tcpClient.remoteIP().toString().c_str());
    }
    tcpReady = (tcpClient && tcpClient.connected());
  }

  lastTcpConnected = tcpReady;

  uint32_t now = millis();
  bool previewActive = ((uint32_t)(now - lastPreviewRequestMs) <= PREVIEW_ACTIVE_WINDOW_MS);
  if (!tcpReady && !previewActive) {
    delay(2);
    return;
  }

  uint16_t targetInterval = tcpReady ? streamFrameIntervalMs : previewFrameIntervalMs;
  if ((uint32_t)(now - lastCaptureMs) < targetInterval) {
    delay(1);
    return;
  }
  lastCaptureMs = now;

  if (!captureLatestFrame()) {
    Serial.println("Capture failed");
    delay(5);
    return;
  }

  if (tcpReady) {
    if (!sendTcpFrameFromCache(tcpClient)) {
      Serial.println("[TCP] CYD client disconnected during send.");
      tcpClient.stop();
      lastTcpConnected = false;
    }
  }
}
