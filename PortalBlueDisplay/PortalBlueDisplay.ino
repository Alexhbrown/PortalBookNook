// CYD ESP32: connect to ESP32-CAM SoftAP and display JPEG frames rotated 90° CW
// Includes:
// - ASCII warm Portal/Hunter Labs boot screen
// - Display-side settings UI (WebServer on port 80)
// - Persistent display tuning settings (Preferences)

#include <ArduinoOTA.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <TFT_eSPI.h>
#include <TJpg_Decoder.h>

// ---------- Wi-Fi (connect to ESP32-CAM AP) ----------
const char* WIFI_SSID = "BluePortal";
const char* WIFI_PASS = "TheCakeIsALie";
const char* OTA_WIFI_SSID = "ahb-IOT";
const char* OTA_WIFI_PASS = "0987654321";
const char* OTA_HOSTNAME = "portal-bluedisplay";
const uint32_t OTA_MODE_TIMEOUT_MS = 10UL * 60UL * 1000UL;

// ESP32-CAM SoftAP default IP is usually 192.168.4.1
const char* CAM_HOST = "192.168.4.1";
const uint16_t CAM_PORT = 5000;

// TFT + JPEG
TFT_eSPI tft = TFT_eSPI();
WiFiClient camClient;
WebServer uiServer(80);
Preferences prefs;

// JPEG buffer (enough for QVGA JPEG)
static uint8_t* jpgBuf = nullptr;
static const size_t JPG_BUF_SIZE = 80 * 1024;

struct DisplaySettings {
  uint8_t decodeEveryN;
  uint16_t headerTimeoutMs;
  uint16_t payloadTimeoutMs;
  uint16_t reconnectDelayMs;
  uint8_t frameCapFps;
  uint8_t liveRotation;
  uint8_t invertDisplay;
  uint8_t showStats;
};

DisplaySettings displaySettings;
bool uiServerStarted = false;

static uint32_t frameCounter = 0;
static uint32_t statsFrames = 0;
static float decodeFps = 0.0f;
static uint32_t statsWindowStartMs = 0;
static uint32_t lastDrawMs = 0;
static uint32_t lastDrawStartMs = 0;

static const uint16_t COLOR_WARM = 0xFD20;

static bool otaMode = false;
static bool otaReady = false;
static uint32_t otaModeStartMs = 0;
static uint32_t otaReconnectAttemptMs = 0;

static bool restartScheduled = false;
static uint32_t restartAtMs = 0;

int clampInt(int value, int minValue, int maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

void scheduleRestart(uint32_t delayMs = 350) {
  restartScheduled = true;
  restartAtMs = millis() + delayMs;
}

bool restartDue() {
  if (!restartScheduled) return false;
  return (int32_t)(millis() - restartAtMs) >= 0;
}

DisplaySettings defaultDisplaySettings() {
  DisplaySettings s = {};
  s.decodeEveryN = 1;
  s.headerTimeoutMs = 5000;
  s.payloadTimeoutMs = 5000;
  s.reconnectDelayMs = 300;
  s.frameCapFps = 0;   // 0 = uncapped
  s.liveRotation = 3;
  s.invertDisplay = 1;
  s.showStats = 1;
  return s;
}

void clampDisplaySettings() {
  displaySettings.decodeEveryN = (uint8_t)clampInt(displaySettings.decodeEveryN, 1, 6);
  displaySettings.headerTimeoutMs = (uint16_t)clampInt(displaySettings.headerTimeoutMs, 1000, 12000);
  displaySettings.payloadTimeoutMs = (uint16_t)clampInt(displaySettings.payloadTimeoutMs, 1000, 12000);
  displaySettings.reconnectDelayMs = (uint16_t)clampInt(displaySettings.reconnectDelayMs, 100, 5000);
  displaySettings.frameCapFps = (uint8_t)clampInt(displaySettings.frameCapFps, 0, 30);
  displaySettings.liveRotation = (uint8_t)clampInt(displaySettings.liveRotation, 0, 3);
  displaySettings.invertDisplay = (uint8_t)clampInt(displaySettings.invertDisplay, 0, 1);
  displaySettings.showStats = (uint8_t)clampInt(displaySettings.showStats, 0, 1);
}

void loadDisplaySettings() {
  displaySettings = defaultDisplaySettings();
  displaySettings.decodeEveryN = (uint8_t)prefs.getInt("decoden", displaySettings.decodeEveryN);
  displaySettings.headerTimeoutMs = (uint16_t)prefs.getInt("hdrto", displaySettings.headerTimeoutMs);
  displaySettings.payloadTimeoutMs = (uint16_t)prefs.getInt("payto", displaySettings.payloadTimeoutMs);
  displaySettings.reconnectDelayMs = (uint16_t)prefs.getInt("reconms", displaySettings.reconnectDelayMs);
  displaySettings.frameCapFps = (uint8_t)prefs.getInt("fpscap", displaySettings.frameCapFps);
  displaySettings.liveRotation = (uint8_t)prefs.getInt("liverot", displaySettings.liveRotation);
  displaySettings.invertDisplay = (uint8_t)prefs.getInt("invert", displaySettings.invertDisplay);
  displaySettings.showStats = (uint8_t)prefs.getInt("stats", displaySettings.showStats);
  clampDisplaySettings();
}

void saveDisplaySettings() {
  prefs.putInt("decoden", displaySettings.decodeEveryN);
  prefs.putInt("hdrto", displaySettings.headerTimeoutMs);
  prefs.putInt("payto", displaySettings.payloadTimeoutMs);
  prefs.putInt("reconms", displaySettings.reconnectDelayMs);
  prefs.putInt("fpscap", displaySettings.frameCapFps);
  prefs.putInt("liverot", displaySettings.liveRotation);
  prefs.putInt("invert", displaySettings.invertDisplay);
  prefs.putInt("stats", displaySettings.showStats);
}

uint8_t getBootRotation() {
  return (uint8_t)((displaySettings.liveRotation + 1) & 0x03);
}

void applyLiveDisplaySettings() {
  tft.setRotation(displaySettings.liveRotation);
  tft.invertDisplay(displaySettings.invertDisplay != 0);
}

int bootStatusY() {
  int y = tft.height() - 28;
  return (y < 0) ? 0 : y;
}

void drawBootStatusArea(const char* phase, const char* detail = nullptr, bool redrawPhase = true) {
  int statusY = bootStatusY();
  tft.setTextFont(1);
  tft.setTextSize(1);
  tft.setTextColor(COLOR_WARM, TFT_BLACK);

  if (redrawPhase) {
    tft.fillRect(0, statusY, tft.width(), 28, TFT_BLACK);
    tft.setCursor(2, statusY);
    tft.print(" STATUS:");
    tft.setCursor(2, statusY + 10);
    tft.print(phase ? phase : "");
  }

  tft.fillRect(0, statusY + 20, tft.width(), 8, TFT_BLACK);
  if (detail) {
    tft.setCursor(2, statusY + 20);
    tft.print(detail);
  }
}

void drawBootScreen(const char* phase, const char* detail = nullptr) {
  static const char* art[] = {
    " .--------------------------------------.",
    " | APERTURE SCIENCE :: PORTAL LINK NODE |",
    " | HUNTER LABS FIELD TERMINAL           |",
    " '--------------------------------------'",
    "",
    "                 /\\                    ",
    "            ____/  \\____               ",
    "           /   HUNTER   \\              ",
    "          /--------------\\             ",
    "          \\    /\\  /\\    /             ",
    "           \\__/  \\/  \\__/              ",
    "           /  \\  /\\  /  \\              ",
    "          /    \\/LABS\\/    \\           ",
    "          \\____      ____/             ",
    "               \\____/                  ",
    "",
    "            [there will be cake]       ",
    "               ,:/+/-                  ",
    "               /M/                     ",
    "          .:/= ;MH/,    ,=/+%XH@MM#@:  ",
    "         -$##@+$###@H@MMM#######H:.    ",
    "      ,/H@H#####H###H###########@+-    ",
    "        '.+####################+.      ",
    "",
    " [WARM BOOT] chamber display core online"
  };

  tft.setRotation(getBootRotation());
  tft.invertDisplay(displaySettings.invertDisplay != 0);
  tft.fillScreen(TFT_BLACK);
  tft.setTextFont(1);
  tft.setTextSize(1);
  tft.setTextColor(COLOR_WARM, TFT_BLACK);

  int y = 2;
  for (size_t i = 0; i < sizeof(art) / sizeof(art[0]); i++) {
    tft.setCursor(2, y);
    tft.println(art[i]);
    y += 8;
  }
  drawBootStatusArea(phase, detail, true);
}

void setOtaModeFlag(bool enabled) {
  prefs.putBool("otamode", enabled);
}

bool connectOtaStaBlocking(uint32_t timeoutMs) {
  if (WiFi.status() == WL_CONNECTED) return true;

  drawBootScreen("OTA mode: joining ahb-IOT...");
  WiFi.disconnect(true, true);
  delay(80);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(OTA_WIFI_SSID, OTA_WIFI_PASS);

  const char spinner[4] = {'|', '/', '-', '\\'};
  uint8_t spinIndex = 0;
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeoutMs) {
    char detail[40];
    snprintf(detail, sizeof(detail), "OTA link attempt %c", spinner[spinIndex++ & 0x03]);
    drawBootStatusArea(nullptr, detail, false);
    delay(250);
  }

  if (WiFi.status() != WL_CONNECTED) {
    drawBootScreen("OTA mode: Wi-Fi retry", "Waiting for ahb-IOT...");
    Serial.println("[OTA] Wi-Fi connect timeout.");
    return false;
  }

  char detail[48];
  snprintf(detail, sizeof(detail), "OTA UI http://%s", WiFi.localIP().toString().c_str());
  drawBootScreen("OTA mode active.", detail);
  Serial.printf("[OTA] Wi-Fi connected, IP: %s\n", WiFi.localIP().toString().c_str());
  return true;
}

void beginArduinoOtaIfReady() {
  if (otaReady || WiFi.status() != WL_CONNECTED) return;

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
  otaReady = true;
  Serial.printf("[OTA] Ready. Hostname: %s, IP: %s, Port: 3232\n",
                OTA_HOSTNAME, WiFi.localIP().toString().c_str());
}

// ----- TJpg callback -----
bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap) {
  if (y >= tft.height()) return 0;
  tft.pushImage(x, y, w, h, bitmap);
  return 1;
}

void drawStatsOverlay() {
  if (!displaySettings.showStats) return;
  tft.fillRect(0, 0, tft.width(), 12, TFT_BLACK);
  tft.setTextFont(1);
  tft.setTextSize(1);
  tft.setTextColor(COLOR_WARM, TFT_BLACK);
  tft.setCursor(2, 2);
  tft.printf("FPS:%2.1f Draw:%lums Skip:%u", decodeFps, (unsigned long)lastDrawMs, (unsigned)displaySettings.decodeEveryN);
}

void updateStats(uint32_t drawMs) {
  lastDrawMs = drawMs;
  statsFrames++;

  uint32_t now = millis();
  if (statsWindowStartMs == 0) statsWindowStartMs = now;

  uint32_t elapsed = now - statsWindowStartMs;
  if (elapsed >= 1000) {
    decodeFps = (float)statsFrames * 1000.0f / (float)elapsed;
    statsFrames = 0;
    statsWindowStartMs = now;
  }
}

void sendDisplayStatus() {
  uint32_t remainingSec = 0;
  if (otaMode) {
    uint32_t elapsed = millis() - otaModeStartMs;
    if (elapsed < OTA_MODE_TIMEOUT_MS) {
      remainingSec = (OTA_MODE_TIMEOUT_MS - elapsed) / 1000U;
    }
  }

  String json = "{";
  json += "\"decode_every_n\":" + String(displaySettings.decodeEveryN) + ",";
  json += "\"header_timeout_ms\":" + String(displaySettings.headerTimeoutMs) + ",";
  json += "\"payload_timeout_ms\":" + String(displaySettings.payloadTimeoutMs) + ",";
  json += "\"reconnect_delay_ms\":" + String(displaySettings.reconnectDelayMs) + ",";
  json += "\"frame_cap_fps\":" + String(displaySettings.frameCapFps) + ",";
  json += "\"live_rotation\":" + String(displaySettings.liveRotation) + ",";
  json += "\"boot_rotation\":" + String(getBootRotation()) + ",";
  json += "\"invert_display\":" + String(displaySettings.invertDisplay) + ",";
  json += "\"show_stats\":" + String(displaySettings.showStats) + ",";
  json += "\"cam_connected\":" + String(camClient.connected() ? 1 : 0) + ",";
  json += "\"decode_fps\":" + String(decodeFps, 1) + ",";
  json += "\"draw_ms\":" + String((unsigned long)lastDrawMs) + ",";
  json += "\"ota_mode\":" + String(otaMode ? 1 : 0) + ",";
  json += "\"ota_timeout_sec\":" + String((unsigned long)remainingSec) + ",";
  json += "\"hostname\":\"" + String(OTA_HOSTNAME) + "\",";
  json += "\"wifi_ssid\":\"" + WiFi.SSID() + "\",";
  json += "\"ip\":\"" + WiFi.localIP().toString() + "\"";
  json += "}";
  uiServer.send(200, "application/json", json);
}

void handleUiControl() {
  if (!uiServer.hasArg("var") || !uiServer.hasArg("val")) {
    uiServer.send(400, "text/plain", "Missing var/val");
    return;
  }

  String var = uiServer.arg("var");
  int val = uiServer.arg("val").toInt();
  bool changed = true;

  if (var == "decode_every_n") {
    displaySettings.decodeEveryN = (uint8_t)clampInt(val, 1, 6);
  } else if (var == "header_timeout_ms") {
    displaySettings.headerTimeoutMs = (uint16_t)clampInt(val, 1000, 12000);
  } else if (var == "payload_timeout_ms") {
    displaySettings.payloadTimeoutMs = (uint16_t)clampInt(val, 1000, 12000);
  } else if (var == "reconnect_delay_ms") {
    displaySettings.reconnectDelayMs = (uint16_t)clampInt(val, 100, 5000);
  } else if (var == "frame_cap_fps") {
    displaySettings.frameCapFps = (uint8_t)clampInt(val, 0, 30);
  } else if (var == "live_rotation") {
    displaySettings.liveRotation = (uint8_t)clampInt(val, 0, 3);
    applyLiveDisplaySettings();
  } else if (var == "invert_display") {
    displaySettings.invertDisplay = (uint8_t)clampInt(val, 0, 1);
    applyLiveDisplaySettings();
  } else if (var == "show_stats") {
    displaySettings.showStats = (uint8_t)clampInt(val, 0, 1);
  } else {
    changed = false;
  }

  if (!changed) {
    uiServer.send(400, "text/plain", "Unknown setting");
    return;
  }

  saveDisplaySettings();
  sendDisplayStatus();
}

void handleUiReset() {
  displaySettings = defaultDisplaySettings();
  applyLiveDisplaySettings();
  saveDisplaySettings();
  sendDisplayStatus();
}

void handleUiPing() {
  char msg[96];
  snprintf(msg, sizeof(msg), "ok uptime_ms=%lu heap=%u", (unsigned long)millis(), (unsigned)ESP.getFreeHeap());
  uiServer.send(200, "text/plain", msg);
}

void handleUiOtaStart() {
  setOtaModeFlag(true);
  uiServer.send(200, "text/plain", "Rebooting into OTA mode on ahb-IOT...");
  scheduleRestart();
}

void handleUiOtaExit() {
  setOtaModeFlag(false);
  uiServer.send(200, "text/plain", "Exiting OTA mode and rebooting...");
  scheduleRestart();
}

void startUiServer() {
  if (uiServerStarted) return;

  static const char page[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>BlueDisplay Tuning</title>
  <style>
    body { font-family: -apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif; margin: 14px; background:#0b0b0b; color:#ffb347; }
    h2 { margin: 0 0 8px 0; }
    .small { color: #f6c77e; font-size: 13px; margin: 6px 0; }
    .row { margin: 8px 0; padding: 10px; border-radius: 10px; background: #171717; }
    .line { display:flex; align-items:center; gap:8px; }
    .line input[type=range] { flex:1; }
    .line button { padding:8px 12px; border:0; border-radius:8px; background:#ff9f1c; color:#111; font-weight:700; }
  </style>
</head>
<body>
  <h2>BlueDisplay Tuning</h2>
  <div class="small">Open while connected to BluePortal. This adjusts CYD display behavior (decode, orientation, and display FPS cap).</div>
  <div id="statusLine" class="small">Loading...</div>

  <div class="row line">
    <button onclick="refreshStatus()">Refresh</button>
    <button onclick="resetDefaults()">Reset Defaults</button>
    <button onclick="enterOtaMode()">Enter OTA Mode (ahb-IOT)</button>
    <button onclick="exitOtaMode()">Exit OTA Mode</button>
  </div>

  <div class="row"><div>Decode Every N Frames</div><div class="line"><input id="decode_every_n" type="range" min="1" max="6"><span id="decode_every_n_v"></span></div></div>
  <div class="row"><div>Display FPS Cap (0 = uncapped)</div><div class="line"><input id="frame_cap_fps" type="range" min="0" max="30"><span id="frame_cap_fps_v"></span></div></div>
  <div class="row"><div>Header Timeout (ms)</div><div class="line"><input id="header_timeout_ms" type="range" min="1000" max="12000" step="100"><span id="header_timeout_ms_v"></span></div></div>
  <div class="row"><div>Payload Timeout (ms)</div><div class="line"><input id="payload_timeout_ms" type="range" min="1000" max="12000" step="100"><span id="payload_timeout_ms_v"></span></div></div>
  <div class="row"><div>Reconnect Delay (ms)</div><div class="line"><input id="reconnect_delay_ms" type="range" min="100" max="5000" step="50"><span id="reconnect_delay_ms_v"></span></div></div>
  <div class="row"><div>Live Rotation</div><div class="line"><input id="live_rotation" type="range" min="0" max="3"><span id="live_rotation_v"></span></div></div>
  <div class="row line"><label><input id="invert_display" type="checkbox"> Invert Display Colors</label></div>
  <div class="row line"><label><input id="show_stats" type="checkbox"> Show FPS Overlay</label></div>

  <script>
    const sliderIds = ["decode_every_n","frame_cap_fps","header_timeout_ms","payload_timeout_ms","reconnect_delay_ms","live_rotation"];
    const checkIds = ["invert_display","show_stats"];

    async function sendControl(id, val) {
      try { await fetch(`/control?var=${id}&val=${val}`); } catch (_) {}
    }

    async function refreshStatus() {
      try {
        const r = await fetch('/status');
        const s = await r.json();
        sliderIds.forEach(id => {
          const el = document.getElementById(id);
          const out = document.getElementById(id + '_v');
          if (!el || !(id in s)) return;
          el.value = s[id];
          if (out) out.textContent = s[id];
        });
        checkIds.forEach(id => {
          const el = document.getElementById(id);
          if (!el || !(id in s)) return;
          el.checked = !!s[id];
        });
        const mode = s.ota_mode ? `OTA(${s.ota_timeout_sec}s left)` : 'RUN';
        document.getElementById('statusLine').textContent = `mode ${mode} | ssid ${s.wifi_ssid} | IP ${s.ip} | cam ${s.cam_connected ? 'connected' : 'disconnected'} | fps ${s.decode_fps} | draw ${s.draw_ms}ms | boot rot ${s.boot_rotation}`;
      } catch (_) {
        document.getElementById('statusLine').textContent = 'Status fetch failed';
      }
    }

    async function resetDefaults() {
      await fetch('/reset');
      await refreshStatus();
    }

    async function enterOtaMode() {
      if (!confirm('Reboot display into OTA mode on ahb-IOT?')) return;
      const r = await fetch('/ota/start');
      alert(await r.text());
    }

    async function exitOtaMode() {
      const r = await fetch('/ota/exit');
      alert(await r.text());
    }

    sliderIds.forEach(id => {
      const el = document.getElementById(id);
      const out = document.getElementById(id + '_v');
      if (!el) return;
      el.addEventListener('input', () => { if (out) out.textContent = el.value; });
      el.addEventListener('change', () => sendControl(id, el.value));
    });

    checkIds.forEach(id => {
      const el = document.getElementById(id);
      if (!el) return;
      el.addEventListener('change', () => sendControl(id, el.checked ? 1 : 0));
    });

    refreshStatus();
    setInterval(refreshStatus, 2000);
  </script>
</body>
</html>
)HTML";

  uiServer.on("/", HTTP_GET, [&]() {
    uiServer.send_P(200, "text/html", page);
  });
  uiServer.on("/ping", HTTP_GET, handleUiPing);
  uiServer.on("/status", HTTP_GET, sendDisplayStatus);
  uiServer.on("/control", HTTP_GET, handleUiControl);
  uiServer.on("/reset", HTTP_GET, handleUiReset);
  uiServer.on("/ota/start", HTTP_GET, handleUiOtaStart);
  uiServer.on("/ota/exit", HTTP_GET, handleUiOtaExit);

  // Common captive-portal and icon probes.
  uiServer.on("/favicon.ico", HTTP_GET, []() { uiServer.send(204, "text/plain", ""); });
  uiServer.on("/apple-touch-icon.png", HTTP_GET, []() { uiServer.send(204, "text/plain", ""); });
  uiServer.on("/hotspot-detect.html", HTTP_GET, []() { uiServer.send(204, "text/plain", ""); });
  uiServer.on("/generate_204", HTTP_GET, []() { uiServer.send(204, "text/plain", ""); });
  uiServer.onNotFound([]() { uiServer.send(404, "text/plain", "Not found"); });

  uiServer.begin();
  uiServerStarted = true;

  Serial.println("Display settings UI started on port 80");
  Serial.print("Display UI URL: http://");
  Serial.println(WiFi.localIP());
}

// ----- Network helpers -----
bool readBytesExact(uint8_t* buf, size_t len, uint32_t timeoutMs) {
  size_t got = 0;
  uint32_t start = millis();
  while (got < len && (millis() - start) < timeoutMs) {
    if (uiServerStarted) uiServer.handleClient();

    int avail = camClient.available();
    if (avail > 0) {
      int readLen = camClient.read(buf + got, len - got);
      if (readLen <= 0) break;
      got += (size_t)readLen;
    } else {
      delay(1);
    }
  }
  return (got == len);
}

bool connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!uiServerStarted) startUiServer();
    return true;
  }

  drawBootScreen("Linking to BluePortal...");

  Serial.print("Connecting WiFi to ");
  Serial.println(WIFI_SSID);

  WiFi.disconnect(true, true);
  delay(100);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  const char spinner[4] = {'|', '/', '-', '\\'};
  uint8_t spinIndex = 0;

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    char detail[32];
    snprintf(detail, sizeof(detail), "Link attempt %c", spinner[spinIndex++ & 0x03]);
    drawBootStatusArea(nullptr, detail, false);
    delay(250);
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi connect failed.");
    drawBootScreen("BluePortal link failed. Retrying...");
    return false;
  }

  Serial.print("WiFi OK, IP: ");
  Serial.println(WiFi.localIP());

  char detail[48];
  snprintf(detail, sizeof(detail), "UI http://%s", WiFi.localIP().toString().c_str());
  drawBootScreen("Portal link established.", detail);

  if (!uiServerStarted) startUiServer();
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

void setupOtaMode() {
  otaMode = true;
  otaModeStartMs = millis();
  otaReconnectAttemptMs = 0;
  otaReady = false;
  camClient.stop();

  if (connectOtaStaBlocking(20000)) {
    beginArduinoOtaIfReady();
  }

  if (!uiServerStarted) startUiServer();
}

// ----- Setup -----
void setup() {
  Serial.begin(115200);
  delay(500);

  // TFT init
  tft.init();
  tft.setRotation(3);        // temporary until settings load
  tft.invertDisplay(true);
  displaySettings = defaultDisplaySettings();
  drawBootScreen("Powering portal display core...");

  // JPEG decoder
  TJpgDec.setJpgScale(1);    // Camera sends QVGA (320x240)
  TJpgDec.setSwapBytes(true);
  TJpgDec.setCallback(tft_output);

  jpgBuf = (uint8_t*)malloc(JPG_BUF_SIZE);
  if (!jpgBuf) {
    Serial.println("Failed to allocate JPEG buffer");
    drawBootScreen("JPEG buffer alloc failed.");
    while (true) delay(1000);
  }

  prefs.begin("portaldsp", false);
  loadDisplaySettings();
  applyLiveDisplaySettings();

  otaMode = prefs.getBool("otamode", false);
  if (otaMode) {
    setupOtaMode();
    return;
  }

  connectWiFi();
}

// ----- Main loop -----
void loop() {
  if (restartDue()) {
    ESP.restart();
  }

  if (uiServerStarted) uiServer.handleClient();

  if (otaMode) {
    uint32_t now = millis();
    if (WiFi.status() == WL_CONNECTED) {
      beginArduinoOtaIfReady();
      if (otaReady) {
        ArduinoOTA.handle();
      }
    } else if ((now - otaReconnectAttemptMs) > 5000) {
      otaReconnectAttemptMs = now;
      otaReady = false;
      Serial.printf("[OTA] Reconnecting to %s...\n", OTA_WIFI_SSID);
      WiFi.disconnect(false, false);
      WiFi.begin(OTA_WIFI_SSID, OTA_WIFI_PASS);
      drawBootScreen("OTA mode: Wi-Fi retry", "Waiting for ahb-IOT...");
    }

    if ((now - otaModeStartMs) >= OTA_MODE_TIMEOUT_MS) {
      Serial.println("[OTA] Timeout reached, returning to BluePortal mode.");
      setOtaModeFlag(false);
      scheduleRestart();
    }

    delay(2);
    return;
  }

  // Ensure Wi-Fi connection; retry if it fails
  if (!connectWiFi()) {
    delay(displaySettings.reconnectDelayMs);
    return;
  }

  // Ensure TCP connection to camera; retry if it fails
  if (!connectCamera()) {
    delay(displaySettings.reconnectDelayMs);
    return;
  }

  // --- Read 4-byte length header ---
  uint8_t header[4];
  if (!readBytesExact(header, 4, displaySettings.headerTimeoutMs)) {
    Serial.println("Header read failed, reconnecting...");
    camClient.stop();
    delay(displaySettings.reconnectDelayMs);
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
    delay(displaySettings.reconnectDelayMs);
    return;
  }

  // --- Read JPEG payload ---
  if (!readBytesExact(jpgBuf, jpgLen, displaySettings.payloadTimeoutMs)) {
    Serial.println("JPEG read failed, reconnecting...");
    camClient.stop();
    delay(displaySettings.reconnectDelayMs);
    return;
  }

  frameCounter++;

  if (displaySettings.frameCapFps > 0) {
    uint16_t minIntervalMs = (uint16_t)(1000U / displaySettings.frameCapFps);
    uint32_t now = millis();
    if (lastDrawStartMs != 0 && (uint32_t)(now - lastDrawStartMs) < minIntervalMs) {
      return;
    }
  }

  // Optionally drop some frames to reduce TFT/CPU load
  if (displaySettings.decodeEveryN > 1 && (frameCounter % displaySettings.decodeEveryN) != 0) {
    return;
  }

  applyLiveDisplaySettings();
  uint32_t drawStart = millis();
  lastDrawStartMs = drawStart;

  // Get JPEG size and center it on the rotated screen
  uint16_t w = 0, h = 0;
  TJpgDec.getJpgSize(&w, &h, jpgBuf, jpgLen);

  int16_t x = (tft.width() - w) / 2;
  int16_t y = (tft.height() - h) / 2;
  if (x < 0) x = 0;
  if (y < 0) y = 0;

  TJpgDec.drawJpg(x, y, jpgBuf, jpgLen);

  uint32_t drawMs = millis() - drawStart;
  updateStats(drawMs);
  drawStatsOverlay();
}
