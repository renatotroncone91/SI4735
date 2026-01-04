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
  {"40M", SW_BAND_TYPE, 7000, 7200, 7100, 1, 7100},
  {"20M", SW_BAND_TYPE, 14000, 14350, 14200, 1, 14200},
  {"15M", SW_BAND_TYPE, 21000, 21450, 21100, 1, 21100},
  {"10M", SW_BAND_TYPE, 28000, 29700, 28400, 1, 28400}
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

uint32_t lastModePress = 0;
uint32_t lastBandPress = 0;
uint32_t lastSeekPress = 0;
uint32_t lastEnc1Press = 0;
uint32_t lastEnc2Press = 0;

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

void showStatus() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.print("Mode: ");
  display.print(modeLabel());
  display.print("  Step: ");
  display.print(currentStep());

  display.setCursor(0, 12);
  display.print("Band: ");
  if (currentMode == MODE_FM) {
    display.print("FM");
  } else {
    display.print(amBands[currentBandIdx].name);
  }

  display.setCursor(0, 24);
  display.setTextSize(2);
  if (currentMode == MODE_FM) {
    display.print(currentFrequency / 100.0, 1);
    display.print(" MHz");
  } else {
    display.print(currentFrequency);
    display.print(" kHz");
  }

  display.setCursor(0, 48);
  display.setTextSize(1);
  display.print("BFO: ");
  display.print(currentBfo);
  display.display();
}

void applyMode() {
  if (currentMode == MODE_FM) {
    ssbLoaded = false;
    rx.setFM(fmMin, fmMax, fmCurrent, fmSteps[currentFmStepIdx]);
    rx.setSeekFmLimits(fmMin, fmMax);
    rx.setSeekFmSpacing(fmSteps[currentFmStepIdx]);
    currentFrequency = fmCurrent;
  } else if (currentMode == MODE_AM) {
    ssbLoaded = false;
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
  showStatus();
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
}
