/*
  Radio ESP32 SI4735 con due encoder, tre pulsanti e OLED (senza menu).

  Controlli
  - Encoder 1: VFO (frequenza)
    * Pressione: cambio step di sintonia
  - Encoder 2: BFO (solo SSB)
    * Pressione: cambio step BFO
  - Pulsante MODE: ciclo FM -> AM -> SSB
  - Pulsante BAND: banda successiva in AM/SSB
  - Pulsante SEEK: ricerca automatica (solo FM/AM)

  Collegamenti (ESP32)
  RESET_PIN 12
  Encoder 1: A 26, B 27, SW 14
  Encoder 2: A 25, B 33, SW 23
  I2C: SDA 21, SCL 22
  OLED I2C 128x64: SDA 21, SCL 22 (addr 0x3C)
  Pulsanti: BAND 32, MODE 13, SEEK 15 (INPUT_PULLUP)

  NOTA: sketch basato sulla libreria PU2CLR SI4735.
*/
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SI4735.h>
#include "Rotary.h"
#include <patch_init.h>
#include "config.h"
#include "rds_helpers.h"

const uint16_t ssb_patch_size = sizeof ssb_patch_content;
uint8_t currentBandIdx = 1;
uint16_t fmCurrent = 10000;

uint8_t currentMode = MODE_FM;
uint8_t currentAmStepIdx = 3;
uint8_t currentFmStepIdx = 1;
uint8_t currentBfoStepIdx = 0;
uint8_t currentSideband = LSB;

int16_t currentBfo = 0;
uint16_t currentFrequency = fmCurrent;
uint8_t seekDirection = 1;
bool ssbLoaded = false;
uint8_t currentRssi = 0;
uint8_t currentSnr = 0;
bool currentStereo = false;

RdsState rdsState = {};

uint32_t lastModePress = 0;
uint32_t lastBandPress = 0;
uint32_t lastSeekPress = 0;
uint32_t lastEnc1Press = 0;
uint32_t lastEnc2Press = 0;
uint32_t lastSignalUpdate = 0;

volatile int encoderCount1 = 0;
volatile int encoderCount2 = 0;

Rotary encoder1 = Rotary(ENCODER1_PIN_A, ENCODER1_PIN_B);
Rotary encoder2 = Rotary(ENCODER2_PIN_A, ENCODER2_PIN_B);

SI4735 rx;
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire);
WebServer server(80);

const char indexHtml[] PROGMEM = R"HTML(
<!doctype html>
<html lang="it">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>SI4735 ESP32</title>
  <style>
    body { font-family: Arial, sans-serif; margin: 16px; background: #0b0d10; color: #f5f7fa; }
    .row { display: flex; flex-wrap: wrap; gap: 8px; margin-bottom: 12px; }
    button { padding: 10px 14px; border: 0; border-radius: 6px; background: #2d6cdf; color: #fff; }
    button.secondary { background: #3a3f46; }
    input[type="number"] { width: 120px; padding: 8px; border-radius: 6px; border: 1px solid #444; background: #15181d; color: #fff; }
    .card { background: #15181d; border: 1px solid #2b2f36; padding: 12px; border-radius: 8px; margin-bottom: 12px; }
    .label { color: #9aa4b2; font-size: 12px; text-transform: uppercase; letter-spacing: .08em; }
    .value { font-size: 20px; margin-top: 4px; }
  </style>
</head>
<body>
  <h1>Radio SI4735</h1>
  <div class="card">
    <div class="label">Stato</div>
    <div class="value" id="status">-</div>
  </div>
  <div class="row">
    <button onclick="cmd('mode','FM')">FM</button>
    <button onclick="cmd('mode','AM')">AM</button>
    <button onclick="cmd('mode','SSB')">SSB</button>
    <button class="secondary" onclick="cmd('band','next')">Banda +</button>
    <button class="secondary" onclick="cmd('seek','1')">Seek</button>
  </div>
  <div class="row">
    <button onclick="cmd('freq','up')">Freq +</button>
    <button onclick="cmd('freq','down')">Freq -</button>
    <button onclick="cmd('bfo','up')">BFO +</button>
    <button onclick="cmd('bfo','down')">BFO -</button>
  </div>
  <div class="row">
    <input id="freqValue" type="number" placeholder="Freq (kHz)">
    <button onclick="setFreq()">Imposta</button>
    <button class="secondary" onclick="cmd('step','next')">Step +</button>
    <button class="secondary" onclick="cmd('bfoStep','next')">Step BFO +</button>
  </div>
  <script>
    async function refresh() {
      const res = await fetch('/api/status');
      const data = await res.json();
      document.getElementById('status').textContent =
        `${data.mode} ${data.freq} ${data.unit} | RSSI ${data.rssi} SNR ${data.snr} | BFO ${data.bfo}`;
    }
    async function cmd(key, value) {
      await fetch(`/api/cmd?${key}=${value}`);
      refresh();
    }
    async function setFreq() {
      const val = document.getElementById('freqValue').value;
      if (val) {
        await cmd('freqSet', val);
      }
    }
    refresh();
    setInterval(refresh, 1500);
  </script>
</body>
</html>
)HTML";

void IRAM_ATTR rotaryEncoder1() {
  unsigned char result = encoder1.process();
  if (result == DIR_CW) {
    encoderCount1++;
  } else if (result == DIR_CCW) {
    encoderCount1--;
  }
}

void IRAM_ATTR rotaryEncoder2() {
  unsigned char result = encoder2.process();
  if (result == DIR_CW) {
    encoderCount2++;
  } else if (result == DIR_CCW) {
    encoderCount2--;
  }
}

void loadSSBPatch() {
  rx.setI2CFastModeCustom(400000);
  rx.loadPatch(ssb_patch_content, ssb_patch_size, SSB_BANDWIDTH_IDX);
  rx.setI2CFastModeCustom(100000);
  ssbLoaded = true;
}

const char *modeLabel() {
  if (currentMode == MODE_FM) {
    return "FM";
  }
  if (currentMode == MODE_AM) {
    return "AM";
  }
  return (currentSideband == USB) ? "USB" : "LSB";
}

uint16_t currentStep() {
  return (currentMode == MODE_FM) ? fmSteps[currentFmStepIdx] : amSteps[currentAmStepIdx];
}

void showStatus() {
  char stepText[12];
  char bandText[12];
  char freqText[16];
  char signalText[20];
  char rdsLine[21];
  char bfoText[16];
  int16_t x1, y1;
  uint16_t w, h;
  uint16_t barWidth = min<uint16_t>(static_cast<uint16_t>(currentRssi) * 2, 126);

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(modeLabel());
  display.print(" B:");
  snprintf(bandText, sizeof(bandText), "%s", (currentMode == MODE_FM) ? "FM" : amBands[currentBandIdx].name);
  display.print(bandText);

  snprintf(stepText, sizeof(stepText), "Stp:%u", currentStep());
  display.getTextBounds(stepText, 0, 0, &x1, &y1, &w, &h);
  display.setCursor(128 - w, 0);
  display.print(stepText);

  display.setTextSize(2);
  if (currentMode == MODE_FM) {
    snprintf(freqText, sizeof(freqText), "%.1f MHz", currentFrequency / 100.0);
  } else {
    snprintf(freqText, sizeof(freqText), "%u kHz", currentFrequency);
  }
  display.getTextBounds(freqText, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((128 - w) / 2, 14);
  display.print(freqText);

  display.setTextSize(1);
  if (currentMode == MODE_FM) {
    if (rdsState.station[0] != '\0') {
      display.setTextSize(2);
      display.setCursor(0, 32);
      display.print(rdsState.station);
      display.setTextSize(1);
    }
    buildRdsScrollLine(rdsState, rdsLine, sizeof(rdsLine));
    if (rdsLine[0] != '\0') {
      display.setCursor(0, 52);
      display.print(rdsLine);
    }
  } else {
    snprintf(signalText, sizeof(signalText), "S:%u N:%u", currentRssi, currentSnr);
    display.setCursor(0, 42);
    display.print(signalText);
  }

  if (currentMode == MODE_SSB) {
    snprintf(bfoText, sizeof(bfoText), "BFO:%d", currentBfo);
    display.getTextBounds(bfoText, 0, 0, &x1, &y1, &w, &h);
    display.setCursor(128 - w, 42);
    display.print(bfoText);
  }

  if (currentMode != MODE_FM) {
    display.drawRect(0, 56, 128, 8, SSD1306_WHITE);
    if (barWidth > 0) {
      display.fillRect(1, 57, min<uint16_t>(barWidth, 126), 6, SSD1306_WHITE);
    }
  }
  display.display();
}

void refreshSignalStatus(bool force) {
  uint32_t now = millis();
  if (!force && (now - lastSignalUpdate) < SIGNAL_UPDATE_MS) {
    return;
  }
  lastSignalUpdate = now;
  rx.getCurrentReceivedSignalQuality();
  uint8_t newRssi = rx.getCurrentRSSI();
  uint8_t newSnr = rx.getCurrentSNR();
  bool newStereo = (currentMode == MODE_FM) ? rx.getCurrentPilot() : false;
  if (force || newRssi != currentRssi || newSnr != currentSnr || newStereo != currentStereo) {
    currentRssi = newRssi;
    currentSnr = newSnr;
    currentStereo = newStereo;
    showStatus();
  }
}

void applyMode() {
  if (currentMode == MODE_FM) {
    ssbLoaded = false;
    rx.setFM(fmMin, fmMax, fmCurrent, fmSteps[currentFmStepIdx]);
    rx.setSeekFmLimits(fmMin, fmMax);
    rx.setSeekFmSpacing(fmSteps[currentFmStepIdx]);
    rx.setRdsConfig(1, 3, 3, 3, 3);
    rx.setFifoCount(1);
    currentFrequency = fmCurrent;
    resetRdsState(rdsState);
  } else if (currentMode == MODE_AM) {
    ssbLoaded = false;
    resetRdsState(rdsState);
    Band &band = amBands[currentBandIdx];
    rx.setAM(band.minFreq, band.maxFreq, band.currentFreq, amSteps[currentAmStepIdx]);
    if (band.bandType == SW_BAND_TYPE) {
      rx.setTuneFrequencyAntennaCapacitor(1);
    } else {
      rx.setTuneFrequencyAntennaCapacitor(0);
    }
    rx.setSeekAmLimits(band.minFreq, band.maxFreq);
    rx.setSeekAmSpacing(5);
    currentFrequency = band.currentFreq;
  } else {
    Band &band = amBands[currentBandIdx];
    if (!ssbLoaded) {
      loadSSBPatch();
    }
    resetRdsState(rdsState);
    if (band.minFreq >= 10000) {
      currentSideband = USB;
    } else {
      currentSideband = LSB;
    }
    rx.setSSB(band.minFreq, band.maxFreq, band.currentFreq, amSteps[currentAmStepIdx], currentSideband);
    rx.setSSBAutomaticVolumeControl(1);
    rx.setSsbSoftMuteMaxAttenuation(SSB_SOFT_MUTE_MAX_ATT);
    rx.setSSBAudioBandwidth(SSB_BANDWIDTH_IDX);
    if (SSB_BANDWIDTH_IDX == 0 || SSB_BANDWIDTH_IDX == 4 || SSB_BANDWIDTH_IDX == 5) {
      rx.setSSBSidebandCutoffFilter(0);
    } else {
      rx.setSSBSidebandCutoffFilter(1);
    }
    if (band.bandType == SW_BAND_TYPE) {
      rx.setTuneFrequencyAntennaCapacitor(1);
    } else {
      rx.setTuneFrequencyAntennaCapacitor(0);
    }
    currentFrequency = band.currentFreq;
    rx.setSSBBfo(currentBfo);
  }
  refreshSignalStatus(true);
}

String currentFrequencyText() {
  if (currentMode == MODE_FM) {
    return String(currentFrequency / 100.0f, 1);
  }
  return String(currentFrequency);
}

void handleStatus() {
  String json = "{";
  json += "\"mode\":\"" + String(modeLabel()) + "\",";
  json += "\"freq\":\"" + currentFrequencyText() + "\",";
  json += "\"unit\":\"" + String((currentMode == MODE_FM) ? "MHz" : "kHz") + "\",";
  json += "\"rssi\":" + String(currentRssi) + ",";
  json += "\"snr\":" + String(currentSnr) + ",";
  json += "\"bfo\":" + String(currentBfo);
  json += "}";
  server.send(200, "application/json", json);
}

void handleCommand() {
  if (server.hasArg("mode")) {
    String mode = server.arg("mode");
    if (mode == "FM") {
      currentMode = MODE_FM;
    } else if (mode == "AM") {
      currentMode = MODE_AM;
    } else if (mode == "SSB") {
      currentMode = MODE_SSB;
      currentBfo = 0;
    }
    applyMode();
  }

  if (server.hasArg("band")) {
    String band = server.arg("band");
    if (band == "next" && currentMode != MODE_FM) {
      currentBandIdx = (currentBandIdx + 1) % amBandCount;
      applyMode();
    }
  }

  if (server.hasArg("seek")) {
    handleSeek();
  }

  if (server.hasArg("freq")) {
    String dir = server.arg("freq");
    if (dir == "up") {
      updateFrequency(1);
    } else if (dir == "down") {
      updateFrequency(-1);
    }
  }

  if (server.hasArg("freqSet")) {
    uint16_t target = server.arg("freqSet").toInt();
    if (currentMode == MODE_FM) {
      target = constrain(target, fmMin, fmMax);
      rx.setFrequency(target);
      fmCurrent = target;
    } else {
      Band &band = amBands[currentBandIdx];
      target = constrain(target, band.minFreq, band.maxFreq);
      rx.setFrequency(target);
      band.currentFreq = target;
    }
    currentFrequency = rx.getFrequency();
    showStatus();
  }

  if (server.hasArg("bfo")) {
    String dir = server.arg("bfo");
    if (dir == "up") {
      updateBfo(1);
    } else if (dir == "down") {
      updateBfo(-1);
    }
  }

  if (server.hasArg("step")) {
    if (currentMode == MODE_FM) {
      currentFmStepIdx = (currentFmStepIdx + 1) % fmStepCount;
      rx.setFrequencyStep(fmSteps[currentFmStepIdx]);
    } else {
      currentAmStepIdx = (currentAmStepIdx + 1) % amStepCount;
      rx.setFrequencyStep(amSteps[currentAmStepIdx]);
    }
    showStatus();
  }

  if (server.hasArg("bfoStep")) {
    currentBfoStepIdx = (currentBfoStepIdx + 1) % bfoStepCount;
    showStatus();
  }

  server.send(200, "text/plain", "OK");
}

void handleRoot() {
  server.send_P(200, "text/html", indexHtml);
}

void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/api/status", handleStatus);
  server.on("/api/cmd", handleCommand);
  server.begin();
}

void setupWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("WiFi...");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < 15000) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi non connesso");
  }
}

void updateFrequency(int8_t direction) {
  if (direction > 0) {
    rx.frequencyUp();
    seekDirection = 1;
  } else {
    rx.frequencyDown();
    seekDirection = 0;
  }
  currentFrequency = rx.getFrequency();
  if (currentMode == MODE_FM) {
    fmCurrent = currentFrequency;
    resetRdsState(rdsState);
    rx.rdsClearFifo();
  } else {
    amBands[currentBandIdx].currentFreq = currentFrequency;
  }
  showStatus();
}

void updateBfo(int8_t direction) {
  if (currentMode != MODE_SSB) {
    return;
  }
  currentBfo += direction * bfoSteps[currentBfoStepIdx];
  rx.setSSBBfo(currentBfo);
  showStatus();
}

void handleSeek() {
  if (currentMode == MODE_SSB) {
    return;
  }
  rx.seekStationProgress(nullptr, seekDirection);
  currentFrequency = rx.getFrequency();
  if (currentMode == MODE_FM) {
    fmCurrent = currentFrequency;
    resetRdsState(rdsState);
    rx.rdsClearFifo();
  } else {
    amBands[currentBandIdx].currentFreq = currentFrequency;
  }
  showStatus();
}

void setup() {
  Serial.begin(115200);

  pinMode(ENCODER1_PUSH_BUTTON, INPUT_PULLUP);
  pinMode(ENCODER2_PUSH_BUTTON, INPUT_PULLUP);
  pinMode(ENCODER1_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER1_PIN_B, INPUT_PULLUP);
  pinMode(ENCODER2_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER2_PIN_B, INPUT_PULLUP);

  pinMode(BAND_BUTTON_PIN, INPUT_PULLUP);
  pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(SEEK_BUTTON_PIN, INPUT_PULLUP);

  Wire.begin(ESP32_I2C_SDA, ESP32_I2C_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    while (true) {
      delay(1000);
    }
  }
  display.clearDisplay();
  display.display();

  attachInterrupt(digitalPinToInterrupt(ENCODER1_PIN_A), rotaryEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER1_PIN_B), rotaryEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_PIN_A), rotaryEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_PIN_B), rotaryEncoder2, CHANGE);

  rx.setI2CFastModeCustom(100000);
  rx.getDeviceI2CAddress(RESET_PIN);
  rx.setup(RESET_PIN, FM_BAND_TYPE);
  rx.setVolume(63);
  resetRdsState(rdsState);
  applyMode();
  setupWifi();
  setupWebServer();
  Serial.println("SI4735 ready");
}

void loop() {
  if (encoderCount1 != 0) {
    int8_t direction = (encoderCount1 > 0) ? 1 : -1;
    encoderCount1 += (direction > 0) ? -1 : 1;
    updateFrequency(direction);
  }

  if (encoderCount2 != 0) {
    int8_t direction = (encoderCount2 > 0) ? 1 : -1;
    encoderCount2 += (direction > 0) ? -1 : 1;
    updateBfo(direction);
  }

  uint32_t now = millis();

  if (digitalRead(ENCODER1_PUSH_BUTTON) == LOW && (now - lastEnc1Press) > DEBOUNCE_MS) {
    lastEnc1Press = now;
    if (currentMode == MODE_FM) {
      currentFmStepIdx = (currentFmStepIdx + 1) % fmStepCount;
      rx.setFrequencyStep(fmSteps[currentFmStepIdx]);
    } else {
      currentAmStepIdx = (currentAmStepIdx + 1) % amStepCount;
      rx.setFrequencyStep(amSteps[currentAmStepIdx]);
    }
    showStatus();
  }

  if (digitalRead(ENCODER2_PUSH_BUTTON) == LOW && (now - lastEnc2Press) > DEBOUNCE_MS) {
    lastEnc2Press = now;
    currentBfoStepIdx = (currentBfoStepIdx + 1) % bfoStepCount;
    showStatus();
  }

  if (digitalRead(MODE_BUTTON_PIN) == LOW && (now - lastModePress) > DEBOUNCE_MS) {
    lastModePress = now;
    currentMode = (currentMode + 1) % 3;
    if (currentMode == MODE_SSB) {
      currentBfo = 0;
    }
    applyMode();
  }

  if (digitalRead(BAND_BUTTON_PIN) == LOW && (now - lastBandPress) > DEBOUNCE_MS) {
    lastBandPress = now;
    if (currentMode != MODE_FM) {
      currentBandIdx = (currentBandIdx + 1) % amBandCount;
      applyMode();
    }
  }

  if (digitalRead(SEEK_BUTTON_PIN) == LOW && (now - lastSeekPress) > DEBOUNCE_MS) {
    lastSeekPress = now;
    handleSeek();
  }

  server.handleClient();
  refreshSignalStatus(false);
  if (currentMode == MODE_FM) {
    if (refreshRdsStatus(rx, rdsState, false)) {
      showStatus();
    }
    if (updateRdsScroll(rdsState)) {
      showStatus();
    }
  }
}
