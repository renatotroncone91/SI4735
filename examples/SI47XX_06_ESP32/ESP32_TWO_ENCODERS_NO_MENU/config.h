#ifndef ESP32_TWO_ENCODERS_CONFIG_H
#define ESP32_TWO_ENCODERS_CONFIG_H

#include <Arduino.h>

// Pin ESP32
#define RESET_PIN 12

// Encoder
#define ENCODER1_PIN_A 26
#define ENCODER1_PIN_B 27
#define ENCODER1_PUSH_BUTTON 14

#define ENCODER2_PIN_A 25
#define ENCODER2_PIN_B 33
#define ENCODER2_PUSH_BUTTON 23

// I2C ESP32
#define ESP32_I2C_SDA 21
#define ESP32_I2C_SCL 22

// Pulsanti (pull-up interna)
#define BAND_BUTTON_PIN 32
#define MODE_BUTTON_PIN 13
#define SEEK_BUTTON_PIN 15

// OLED
#define OLED_WIDTH 128
#define OLED_HEIGHT 64

// WiFi
#define WIFI_SSID "Casa"
#define WIFI_PASSWORD "04071991"

// Tipi banda
#define FM_BAND_TYPE 0
#define MW_BAND_TYPE 1
#define SW_BAND_TYPE 2
#define LW_BAND_TYPE 3

// Modalita'
#define MODE_FM 0
#define MODE_AM 1
#define MODE_SSB 2

#define LSB 1
#define USB 2

// Tempi
#define DEBOUNCE_MS 200
#define SIGNAL_UPDATE_MS 400
#define RDS_UPDATE_MS 400
#define RDS_SCROLL_MS 500
#define RDS_SCROLL_STEP 2

// Parametri SSB
#define SSB_BANDWIDTH_IDX 2
#define SSB_SOFT_MUTE_MAX_ATT 0

struct Band {
  const char *name;
  uint8_t bandType;
  uint16_t minFreq;
  uint16_t maxFreq;
  uint16_t defaultFreq;
  uint8_t defaultStepIdx;
  uint16_t currentFreq;
};

extern Band amBands[];
extern const uint8_t amBandCount;

extern const uint16_t fmMin;
extern const uint16_t fmMax;

extern const uint16_t amSteps[];
extern const uint8_t amStepCount;

extern const uint16_t fmSteps[];
extern const uint8_t fmStepCount;

extern const int16_t bfoSteps[];
extern const uint8_t bfoStepCount;

#endif
