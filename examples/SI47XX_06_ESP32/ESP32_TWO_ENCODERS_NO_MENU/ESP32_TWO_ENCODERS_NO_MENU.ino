/*
  ESP32 sketch: simple control for SI4735 with two encoders + 3 buttons (no menu).

  Controls
  - Encoder 1: VFO (frequency)
    * Push: cycle tuning step
  - Encoder 2: BFO (SSB only)
    * Push: cycle BFO step
  - MODE button: cycle FM -> AM -> SSB
  - BAND button: next AM/SSB band
  - SEEK button: auto seek (FM/AM only)

  Wiring (ESP32)
  RESET_PIN 12
  Encoder 1: A 26, B 27, SW 14
  Encoder 2: A 25, B 33, SW 23
  I2C: SDA 21, SCL 22
  OLED I2C 128x64: SDA 21, SCL 22 (addr 0x3C)
  Buttons: BAND 32, MODE 13, SEEK 15 (INPUT_PULLUP)

  NOTE: This sketch uses the PU2CLR SI4735 library.
*/
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SI4735.h>
#include "Rotary.h"
#include <patch_init.h>

#define RESET_PIN 12

// Encoder PINs
#define ENCODER1_PIN_A 26
#define ENCODER1_PIN_B 27
#define ENCODER1_PUSH_BUTTON 14

#define ENCODER2_PIN_A 25
#define ENCODER2_PIN_B 33
#define ENCODER2_PUSH_BUTTON 23

// I2C bus pin on ESP32
#define ESP32_I2C_SDA 21
#define ESP32_I2C_SCL 22

// Buttons (internal pull-ups)
#define BAND_BUTTON_PIN 32
#define MODE_BUTTON_PIN 13
#define SEEK_BUTTON_PIN 15

#define OLED_WIDTH 128
#define OLED_HEIGHT 64

#define FM_BAND_TYPE 0
#define MW_BAND_TYPE 1
#define SW_BAND_TYPE 2
#define LW_BAND_TYPE 3

#define MODE_FM 0
#define MODE_AM 1
#define MODE_SSB 2

#define LSB 1
#define USB 2

#define DEBOUNCE_MS 200
#define SIGNAL_UPDATE_MS 400
#define RDS_UPDATE_MS 400
#define RDS_SCROLL_MS 600

const uint16_t ssb_patch_size = sizeof ssb_patch_content;

struct Band {
  const char *name;
  uint8_t bandType;
  uint16_t minFreq;
  uint16_t maxFreq;
  uint16_t defaultFreq;
  uint8_t defaultStepIdx;
  uint16_t currentFreq;
};

Band amBands[] = {
  {"LW", LW_BAND_TYPE, 150, 283, 198, 0, 198},
  {"MW", MW_BAND_TYPE, 520, 1710, 1000, 3, 1000},
  {"MW-EU", MW_BAND_TYPE, 531, 1701, 783, 2, 783},
  {"160M", MW_BAND_TYPE, 1700, 3500, 2500, 1, 2500},
  {"80M", SW_BAND_TYPE, 3500, 4000, 3700, 1, 3700},
  {"SW4-5", SW_BAND_TYPE, 4000, 5500, 4885, 1, 4885},
  {"SW5-6", SW_BAND_TYPE, 5500, 6500, 6000, 1, 6000},
  {"40M", SW_BAND_TYPE, 6500, 7300, 7100, 1, 7100},
  {"SW7-8", SW_BAND_TYPE, 7200, 8000, 7200, 1, 7200},
  {"SW9-11", SW_BAND_TYPE, 9000, 11000, 9500, 1, 9500},
  {"SW11-13", SW_BAND_TYPE, 11100, 13000, 11900, 1, 11900},
  {"SW13-14", SW_BAND_TYPE, 13000, 14000, 13500, 1, 13500},
  {"20M", SW_BAND_TYPE, 14000, 15000, 14200, 1, 14200},
  {"SW15-17", SW_BAND_TYPE, 15000, 17000, 15300, 1, 15300},
  {"SW17-18", SW_BAND_TYPE, 17000, 18000, 17500, 1, 17500},
  {"15M", SW_BAND_TYPE, 20000, 21400, 21100, 1, 21100},
  {"SW21-22", SW_BAND_TYPE, 21400, 22800, 21500, 1, 21500},
  {"CB", SW_BAND_TYPE, 26000, 28000, 27500, 1, 27500},
  {"10M", SW_BAND_TYPE, 28000, 30000, 28400, 1, 28400},
  {"ALL", SW_BAND_TYPE, 150, 30000, 15000, 3, 15000}
};

const uint8_t amBandCount = sizeof(amBands) / sizeof(amBands[0]);
uint8_t currentBandIdx = 1;

const uint16_t fmMin = 6400;
const uint16_t fmMax = 10800;
uint16_t fmCurrent = 10000;

const uint16_t amSteps[] = {1, 5, 9, 10, 50, 100};
const uint8_t amStepCount = sizeof(amSteps) / sizeof(amSteps[0]);

const uint16_t fmSteps[] = {5, 10, 50, 100};
const uint8_t fmStepCount = sizeof(fmSteps) / sizeof(fmSteps[0]);

const int16_t bfoSteps[] = {10, 50, 100};
const uint8_t bfoStepCount = sizeof(bfoSteps) / sizeof(bfoSteps[0]);

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
bool rdsSynced = false;
char rdsStation[9] = "";
char rdsText[65] = "";
uint8_t rdsScrollIndex = 0;

uint32_t lastModePress = 0;
uint32_t lastBandPress = 0;
uint32_t lastSeekPress = 0;
uint32_t lastEnc1Press = 0;
uint32_t lastEnc2Press = 0;
uint32_t lastSignalUpdate = 0;
uint32_t lastRdsUpdate = 0;
uint32_t lastRdsScroll = 0;

volatile int encoderCount1 = 0;
volatile int encoderCount2 = 0;

Rotary encoder1 = Rotary(ENCODER1_PIN_A, ENCODER1_PIN_B);
Rotary encoder2 = Rotary(ENCODER2_PIN_A, ENCODER2_PIN_B);

SI4735 rx;
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire);

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
  rx.loadPatch(ssb_patch_content, ssb_patch_size, 2);
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

void sanitizeRdsText(char *text) {
  if (!text) {
    return;
  }
  for (size_t i = 0; text[i] != '\0'; i++) {
    if (text[i] < 32) {
      text[i] = ' ';
    }
  }
}

void clearRdsData() {
  rdsStation[0] = '\0';
  rdsText[0] = '\0';
  rdsScrollIndex = 0;
  rdsSynced = false;
}

void showStatus() {
  char stepText[12];
  char bandText[12];
  char freqText[16];
  char signalText[20];
  char signalRightText[12];
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
    snprintf(signalText, sizeof(signalText), "S:%u N:%u", currentRssi, currentSnr);
    snprintf(signalRightText, sizeof(signalRightText), "%s", currentStereo ? "ST" : "MO");
    display.setCursor(0, 34);
    display.print(signalText);
    display.getTextBounds(signalRightText, 0, 0, &x1, &y1, &w, &h);
    display.setCursor(128 - w, 34);
    display.print(signalRightText);

    if (rdsText[0] != '\0') {
      size_t len = strlen(rdsText);
      if (len <= 20) {
        snprintf(rdsLine, sizeof(rdsLine), "%s", rdsText);
      } else {
        for (uint8_t i = 0; i < 20; i++) {
          rdsLine[i] = rdsText[(rdsScrollIndex + i) % len];
        }
        rdsLine[20] = '\0';
      }
      display.setCursor(0, 48);
      display.print(rdsLine);
    } else if (rdsStation[0] != '\0') {
      display.setCursor(0, 48);
      display.print(rdsStation);
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

void refreshRdsStatus(bool force) {
  if (currentMode != MODE_FM) {
    return;
  }
  uint32_t now = millis();
  if (!force && (now - lastRdsUpdate) < RDS_UPDATE_MS) {
    return;
  }
  lastRdsUpdate = now;

  rx.getRdsStatus();
  if (!rx.getRdsReceived()) {
    return;
  }
  if (!rx.getRdsSync() || rx.getNumRdsFifoUsed() == 0) {
    if (rdsSynced) {
      rdsSynced = false;
      showStatus();
    }
    return;
  }

  bool updated = false;
  rdsSynced = true;

  char *stationName = rx.getRdsStationName();
  if (stationName != nullptr) {
    sanitizeRdsText(stationName);
    if (strncmp(rdsStation, stationName, sizeof(rdsStation) - 1) != 0) {
      snprintf(rdsStation, sizeof(rdsStation), "%s", stationName);
      updated = true;
    }
  }

  char *programInfo = rx.getRdsProgramInformation();
  if (programInfo != nullptr) {
    sanitizeRdsText(programInfo);
    if (strncmp(rdsText, programInfo, sizeof(rdsText) - 1) != 0) {
      snprintf(rdsText, sizeof(rdsText), "%s", programInfo);
      rdsScrollIndex = 0;
      updated = true;
    }
  }

  if (updated) {
    showStatus();
  }
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
    clearRdsData();
  } else if (currentMode == MODE_AM) {
    ssbLoaded = false;
    clearRdsData();
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
    clearRdsData();
    if (band.minFreq >= 10000) {
      currentSideband = USB;
    } else {
      currentSideband = LSB;
    }
    rx.setSSB(band.minFreq, band.maxFreq, band.currentFreq, amSteps[currentAmStepIdx], currentSideband);
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
    clearRdsData();
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
    clearRdsData();
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
  applyMode();
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

  refreshSignalStatus(false);
  refreshRdsStatus(false);
  if (currentMode == MODE_FM && rdsText[0] != '\0') {
    uint32_t nowScroll = millis();
    if ((nowScroll - lastRdsScroll) > RDS_SCROLL_MS) {
      lastRdsScroll = nowScroll;
      if (strlen(rdsText) > 20) {
        rdsScrollIndex = (rdsScrollIndex + 1) % strlen(rdsText);
        showStatus();
      }
    }
  }
}
