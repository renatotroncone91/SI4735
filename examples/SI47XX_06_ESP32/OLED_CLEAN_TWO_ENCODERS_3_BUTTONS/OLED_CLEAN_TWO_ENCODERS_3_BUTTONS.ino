/*
  ESP32 + SI4735 + OLED 128x64 (I2C) + 2 encoders + 3 buttons

  - Encoder 1: tuning (push = step size, long press = bandwidth)
  - Encoder 2: volume (push = mute)
  - Button MODE: cycle FM -> AM -> SSB
  - Button BAND: change band (AM/SSB). In FM it cycles FM band list (if more than one)
  - Button SEEK: short press seek up; long press seek down. In SSB: short toggles USB/LSB, long toggles BFO control

  Pins are set for the user wiring below.

  Sketch based on PU2CLR examples, simplified for a clean UI (no menus).
*/
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SI4735.h>
#include <patch_init.h>
#include "Rotary.h"
#include "DSEG7_Classic_Regular_16.h"

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

#define FM_BAND_TYPE 0
#define MW_BAND_TYPE 1
#define SW_BAND_TYPE 2
#define LW_BAND_TYPE 3

#define FM 0
#define LSB 1
#define USB 2
#define AM 3

#define DEFAULT_VOLUME 35
#define LONG_PRESS_MS 800
#define DEBOUNCE_MS 30
#define BUTTON_ACTIVE_LEVEL LOW

const uint16_t size_content = sizeof ssb_patch_content;

struct Band
{
  const char *name;
  uint8_t type;
  uint16_t minFreq;
  uint16_t maxFreq;
  uint16_t defaultFreq;
  uint8_t stepIdx;
  uint8_t bwIdx;
};

Band bandTable[] = {
    {"FM", FM_BAND_TYPE, 6400, 10800, 10390, 1, 0},
    {"MW", MW_BAND_TYPE, 522, 1710,  810, 3, 4},
    {"LW", LW_BAND_TYPE, 150,  520,  198, 2, 4},
    {"80", MW_BAND_TYPE, 3500, 4000, 3700, 0, 4},
    {"40", SW_BAND_TYPE, 7000, 7300, 7100, 0, 4},
    {"31", SW_BAND_TYPE, 9400, 9900, 9700, 1, 4},
    {"25", SW_BAND_TYPE, 11600, 12100, 11800, 1, 4},
    {"19", SW_BAND_TYPE, 15000, 15800, 15500, 1, 4},
    {"16", SW_BAND_TYPE, 17400, 17900, 17600, 1, 4},
    {"13", SW_BAND_TYPE, 21400, 21900, 21600, 1, 4},
};

const uint8_t bandCount = sizeof(bandTable) / sizeof(bandTable[0]);

int tabAmStep[] = {1, 5, 9, 10, 50, 100};
int tabFmStep[] = {5, 10, 20};
const uint8_t maxAmStep = (sizeof(tabAmStep) / sizeof(tabAmStep[0])) - 1;
const uint8_t maxFmStep = (sizeof(tabFmStep) / sizeof(tabFmStep[0])) - 1;

struct Bandwidth
{
  uint8_t idx;
  const char *desc;
};

Bandwidth bandwidthSSB[] = {
    {4, "0.5"},
    {5, "1.0"},
    {0, "1.2"},
    {1, "2.2"},
    {2, "3.0"},
    {3, "4.0"}};

Bandwidth bandwidthAM[] = {
    {4, "1.0"},
    {5, "1.8"},
    {3, "2.0"},
    {6, "2.5"},
    {2, "3.0"},
    {1, "4.0"},
    {0, "6.0"}};

Bandwidth bandwidthFM[] = {
    {0, "AUT"},
    {1, "110"},
    {2, " 84"},
    {3, " 60"},
    {4, " 40"}};

const uint8_t maxSsbBw = (sizeof(bandwidthSSB) / sizeof(bandwidthSSB[0])) - 1;
const uint8_t maxAmBw = (sizeof(bandwidthAM) / sizeof(bandwidthAM[0])) - 1;
const uint8_t maxFmBw = (sizeof(bandwidthFM) / sizeof(bandwidthFM[0])) - 1;

struct ButtonState
{
  uint8_t pin;
  bool lastReading;
  bool pressed;
  unsigned long lastChange;
  unsigned long pressedAt;
};

enum ButtonEvent
{
  BUTTON_NONE,
  BUTTON_SHORT,
  BUTTON_LONG
};

ButtonState modeButton{MODE_BUTTON_PIN, HIGH, false, 0, 0};
ButtonState bandButton{BAND_BUTTON_PIN, HIGH, false, 0, 0};
ButtonState seekButton{SEEK_BUTTON_PIN, HIGH, false, 0, 0};
ButtonState enc1Button{ENCODER1_PUSH_BUTTON, HIGH, false, 0, 0};
ButtonState enc2Button{ENCODER2_PUSH_BUTTON, HIGH, false, 0, 0};

Rotary encoder1(ENCODER1_PIN_A, ENCODER1_PIN_B);
Rotary encoder2(ENCODER2_PIN_A, ENCODER2_PIN_B);

Adafruit_SSD1306 display(128, 64, &Wire, -1);
SI4735 radio;

volatile int8_t encoderCount1 = 0;
volatile int8_t encoderCount2 = 0;

uint8_t currentMode = FM;
uint8_t ssbSideband = USB;
bool ssbLoaded = false;
bool bfoOn = false;
int16_t currentBFO = 0;

uint8_t bandIdx = 0;
uint8_t currentStepIdx = 1;
uint16_t currentFrequency = 10390;

uint8_t bwIdxSSB = 4;
uint8_t bwIdxAM = 4;
uint8_t bwIdxFM = 0;

uint8_t volumeLevel = DEFAULT_VOLUME;
uint8_t lastVolume = DEFAULT_VOLUME;
bool isMuted = false;

uint8_t rssi = 0;
unsigned long lastRssiUpdate = 0;

void IRAM_ATTR readEncoder1()
{
  uint8_t result = encoder1.process();
  if (result == DIR_CW)
    encoderCount1 = 1;
  else if (result == DIR_CCW)
    encoderCount1 = -1;
}

void IRAM_ATTR readEncoder2()
{
  uint8_t result = encoder2.process();
  if (result == DIR_CW)
    encoderCount2 = 1;
  else if (result == DIR_CCW)
    encoderCount2 = -1;
}

void setupPins()
{
  pinMode(ENCODER1_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER1_PIN_B, INPUT_PULLUP);
  pinMode(ENCODER2_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER2_PIN_B, INPUT_PULLUP);

  pinMode(ENCODER1_PUSH_BUTTON, INPUT_PULLUP);
  pinMode(ENCODER2_PUSH_BUTTON, INPUT_PULLUP);

  pinMode(BAND_BUTTON_PIN, INPUT_PULLUP);
  pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(SEEK_BUTTON_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER1_PIN_A), readEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER1_PIN_B), readEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_PIN_A), readEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_PIN_B), readEncoder2, CHANGE);
}

void setupDisplay()
{
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setFont(NULL);
  display.setCursor(0, 0);
  display.print("SI4735");
  display.setCursor(0, 10);
  display.print("ESP32 RADIO");
  display.display();
  delay(1200);
}

void setupRadio()
{
  radio.setI2CFastModeCustom(100000);
  radio.getDeviceI2CAddress(RESET_PIN);
  radio.setup(RESET_PIN, MW_BAND_TYPE);
  delay(200);
  radio.setVolume(volumeLevel);
  applyBandForMode();
}

void applyBandForMode()
{
  if (currentMode == FM)
  {
    if (bandTable[bandIdx].type != FM_BAND_TYPE)
    {
      for (uint8_t i = 0; i < bandCount; i++)
      {
        if (bandTable[i].type == FM_BAND_TYPE)
        {
          bandIdx = i;
          break;
        }
      }
    }
  }
  else
  {
    if (bandTable[bandIdx].type == FM_BAND_TYPE)
    {
      for (uint8_t i = 0; i < bandCount; i++)
      {
        if (bandTable[i].type != FM_BAND_TYPE)
        {
          bandIdx = i;
          break;
        }
      }
    }
  }

  currentFrequency = bandTable[bandIdx].defaultFreq;
  currentStepIdx = bandTable[bandIdx].stepIdx;
  applyRadioSettings();
}

void loadSSB()
{
  radio.setI2CFastModeCustom(400000);
  radio.loadPatch(ssb_patch_content, size_content, bandwidthSSB[bwIdxSSB].idx);
  radio.setI2CFastModeCustom(100000);
  ssbLoaded = true;
}

void applyRadioSettings()
{
  if (currentMode == FM)
  {
    bfoOn = false;
    radio.setTuneFrequencyAntennaCapacitor(0);
    radio.setFM(bandTable[bandIdx].minFreq, bandTable[bandIdx].maxFreq, currentFrequency, tabFmStep[currentStepIdx]);
    radio.setFmBandwidth(bandwidthFM[bwIdxFM].idx);
    radio.setSeekFmLimits(bandTable[bandIdx].minFreq, bandTable[bandIdx].maxFreq);
    radio.setFrequencyStep(tabFmStep[currentStepIdx]);
  }
  else if (currentMode == AM)
  {
    bfoOn = false;
    radio.setTuneFrequencyAntennaCapacitor((bandTable[bandIdx].type == MW_BAND_TYPE || bandTable[bandIdx].type == LW_BAND_TYPE) ? 0 : 1);
    radio.setAM(bandTable[bandIdx].minFreq, bandTable[bandIdx].maxFreq, currentFrequency, tabAmStep[currentStepIdx]);
    radio.setBandwidth(bandwidthAM[bwIdxAM].idx, 1);
    radio.setSeekAmLimits(bandTable[bandIdx].minFreq, bandTable[bandIdx].maxFreq);
    radio.setFrequencyStep(tabAmStep[currentStepIdx]);
  }
  else
  {
    if (!ssbLoaded)
    {
      loadSSB();
    }
    radio.setTuneFrequencyAntennaCapacitor((bandTable[bandIdx].type == MW_BAND_TYPE || bandTable[bandIdx].type == LW_BAND_TYPE) ? 0 : 1);
    radio.setSSB(bandTable[bandIdx].minFreq, bandTable[bandIdx].maxFreq, currentFrequency, tabAmStep[currentStepIdx], currentMode);
    radio.setSSBAudioBandwidth(bandwidthSSB[bwIdxSSB].idx);
    radio.setSSBBfo(currentBFO);
    radio.setFrequencyStep(tabAmStep[currentStepIdx]);
  }

  currentFrequency = radio.getFrequency();
  updateScreen();
}

void updateScreen()
{
  char freqBuffer[12];
  char infoBuffer[22];
  char stepBuffer[10];

  display.clearDisplay();
  display.setFont(NULL);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(bandTable[bandIdx].name);
  display.print(" ");
  display.print((currentMode == FM) ? "FM" : (currentMode == AM) ? "AM" : (currentMode == USB) ? "USB" : "LSB");

  display.setCursor(74, 0);
  if (currentMode == FM)
    snprintf(stepBuffer, sizeof(stepBuffer), "%2dk", tabFmStep[currentStepIdx]);
  else
    snprintf(stepBuffer, sizeof(stepBuffer), "%3dk", tabAmStep[currentStepIdx]);
  display.print(stepBuffer);
  if (isMuted)
  {
    display.setCursor(110, 0);
    display.print("M");
  }

  display.setFont(&DSEG7_Classic_Regular_16);
  display.setCursor(0, 28);
  if (currentMode == FM)
  {
    float freqMHz = currentFrequency / 100.0f;
    snprintf(freqBuffer, sizeof(freqBuffer), "%6.2f", freqMHz);
  }
  else
  {
    snprintf(freqBuffer, sizeof(freqBuffer), "%6u", currentFrequency);
  }
  display.print(freqBuffer);

  display.setFont(NULL);
  display.setTextSize(1);
  display.setCursor(0, 52);
  snprintf(infoBuffer, sizeof(infoBuffer), "Vol:%2u", volumeLevel);
  display.print(infoBuffer);

  display.setCursor(56, 52);
  if (currentMode == FM)
  {
    snprintf(infoBuffer, sizeof(infoBuffer), "BW:%s", bandwidthFM[bwIdxFM].desc);
  }
  else if (currentMode == AM)
  {
    snprintf(infoBuffer, sizeof(infoBuffer), "BW:%s", bandwidthAM[bwIdxAM].desc);
  }
  else
  {
    snprintf(infoBuffer, sizeof(infoBuffer), "BW:%s", bandwidthSSB[bwIdxSSB].desc);
  }
  display.print(infoBuffer);

  if (bfoOn && (currentMode == USB || currentMode == LSB))
  {
    display.setCursor(0, 40);
    snprintf(infoBuffer, sizeof(infoBuffer), "BFO:%d", currentBFO);
    display.print(infoBuffer);
  }
  else if (currentMode != FM)
  {
    display.setCursor(0, 40);
    snprintf(infoBuffer, sizeof(infoBuffer), "BAND:%s", bandTable[bandIdx].name);
    display.print(infoBuffer);
  }

  display.display();
}

void updateRssi()
{
  if (millis() - lastRssiUpdate < 200)
    return;

  lastRssiUpdate = millis();
  rssi = radio.getCurrentRSSI();
  display.setFont(NULL);
  display.setTextSize(1);
  display.fillRect(96, 8, 32, 8, SSD1306_BLACK);
  display.setCursor(96, 8);
  display.print("S");
  display.print(rssi);
  display.display();
}

void handleEncoder1()
{
  if (encoderCount1 == 0)
    return;

  if (bfoOn && (currentMode == USB || currentMode == LSB))
  {
    currentBFO += (encoderCount1 == 1) ? 10 : -10;
    radio.setSSBBfo(currentBFO);
  }
  else
  {
    if (encoderCount1 == 1)
      radio.frequencyUp();
    else
      radio.frequencyDown();

    currentFrequency = radio.getFrequency();
  }

  encoderCount1 = 0;
  updateScreen();
}

void handleEncoder2()
{
  if (encoderCount2 == 0)
    return;

  if (encoderCount2 == 1)
    radio.volumeUp();
  else
    radio.volumeDown();

  volumeLevel = radio.getVolume();
  encoderCount2 = 0;
  updateScreen();
}

ButtonEvent pollButton(ButtonState &button)
{
  bool reading = digitalRead(button.pin);
  if (reading != button.lastReading)
  {
    button.lastChange = millis();
  }

  if ((millis() - button.lastChange) > DEBOUNCE_MS)
  {
    if (!button.pressed && reading == BUTTON_ACTIVE_LEVEL)
    {
      button.pressed = true;
      button.pressedAt = millis();
    }

    if (button.pressed && reading != BUTTON_ACTIVE_LEVEL)
    {
      button.pressed = false;
      unsigned long pressTime = millis() - button.pressedAt;
      if (pressTime >= LONG_PRESS_MS)
        return BUTTON_LONG;
      return BUTTON_SHORT;
    }
  }

  button.lastReading = reading;
  return BUTTON_NONE;
}

void cycleMode()
{
  if (currentMode == FM)
  {
    currentMode = AM;
  }
  else if (currentMode == AM)
  {
    currentMode = ssbSideband;
  }
  else
  {
    currentMode = FM;
  }

  applyBandForMode();
}

void nextBand()
{
  uint8_t startIdx = bandIdx;
  do
  {
    bandIdx = (bandIdx + 1) % bandCount;
  } while (((currentMode == FM) && bandTable[bandIdx].type != FM_BAND_TYPE) ||
           ((currentMode != FM) && bandTable[bandIdx].type == FM_BAND_TYPE));

  if (bandIdx == startIdx)
    return;

  currentFrequency = bandTable[bandIdx].defaultFreq;
  currentStepIdx = bandTable[bandIdx].stepIdx;
  applyRadioSettings();
}

void seekStation(bool up)
{
  if (currentMode == USB || currentMode == LSB)
    return;

  uint8_t direction = up ? 1 : 0;
  radio.seekStationProgress(showFrequencySeek, direction);
  currentFrequency = radio.getFrequency();
  updateScreen();
}

void showFrequencySeek(uint16_t freq)
{
  currentFrequency = freq;
  updateScreen();
}

void toggleSideband()
{
  if (currentMode == USB)
    currentMode = LSB;
  else if (currentMode == LSB)
    currentMode = USB;
  ssbSideband = currentMode;
  applyRadioSettings();
}

void toggleStep()
{
  if (currentMode == FM)
  {
    currentStepIdx = (currentStepIdx < maxFmStep) ? (currentStepIdx + 1) : 0;
    radio.setFrequencyStep(tabFmStep[currentStepIdx]);
  }
  else
  {
    currentStepIdx = (currentStepIdx < maxAmStep) ? (currentStepIdx + 1) : 0;
    radio.setFrequencyStep(tabAmStep[currentStepIdx]);
  }
  updateScreen();
}

void toggleBandwidth()
{
  if (currentMode == FM)
  {
    bwIdxFM = (bwIdxFM < maxFmBw) ? (bwIdxFM + 1) : 0;
    radio.setFmBandwidth(bandwidthFM[bwIdxFM].idx);
  }
  else if (currentMode == AM)
  {
    bwIdxAM = (bwIdxAM < maxAmBw) ? (bwIdxAM + 1) : 0;
    radio.setBandwidth(bandwidthAM[bwIdxAM].idx, 1);
  }
  else
  {
    bwIdxSSB = (bwIdxSSB < maxSsbBw) ? (bwIdxSSB + 1) : 0;
    radio.setSSBAudioBandwidth(bandwidthSSB[bwIdxSSB].idx);
  }
  updateScreen();
}

void toggleMute()
{
  if (!isMuted)
  {
    lastVolume = volumeLevel;
    radio.setVolume(0);
    volumeLevel = 0;
    isMuted = true;
  }
  else
  {
    radio.setVolume(lastVolume);
    volumeLevel = lastVolume;
    isMuted = false;
  }
  updateScreen();
}

void toggleBfo()
{
  if (currentMode == USB || currentMode == LSB)
  {
    bfoOn = !bfoOn;
  }
  updateScreen();
}

void setup()
{
  setupPins();
  Wire.begin(ESP32_I2C_SDA, ESP32_I2C_SCL);
  setupDisplay();
  setupRadio();
  updateScreen();
}

void loop()
{
  handleEncoder1();
  handleEncoder2();

  ButtonEvent modeEvent = pollButton(modeButton);
  if (modeEvent == BUTTON_SHORT)
  {
    cycleMode();
  }

  ButtonEvent bandEvent = pollButton(bandButton);
  if (bandEvent == BUTTON_SHORT)
  {
    nextBand();
  }

  ButtonEvent seekEvent = pollButton(seekButton);
  if (seekEvent != BUTTON_NONE)
  {
    if (currentMode == USB || currentMode == LSB)
    {
      if (seekEvent == BUTTON_LONG)
        toggleBfo();
      else
        toggleSideband();
    }
    else
    {
      seekStation(seekEvent == BUTTON_SHORT);
    }
  }

  ButtonEvent enc1Event = pollButton(enc1Button);
  if (enc1Event != BUTTON_NONE)
  {
    if (enc1Event == BUTTON_LONG)
      toggleBandwidth();
    else
      toggleStep();
  }

  ButtonEvent enc2Event = pollButton(enc2Button);
  if (enc2Event == BUTTON_SHORT)
  {
    toggleMute();
  }

  updateRssi();
}
