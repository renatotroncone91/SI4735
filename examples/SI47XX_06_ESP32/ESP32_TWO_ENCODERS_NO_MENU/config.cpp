#include "config.h"

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

const uint16_t fmMin = 6400;
const uint16_t fmMax = 10800;

const uint16_t amSteps[] = {1, 5, 9, 10, 50, 100};
const uint8_t amStepCount = sizeof(amSteps) / sizeof(amSteps[0]);

const uint16_t fmSteps[] = {5, 10, 50, 100};
const uint8_t fmStepCount = sizeof(fmSteps) / sizeof(fmSteps[0]);

const int16_t bfoSteps[] = {10, 50, 100};
const uint8_t bfoStepCount = sizeof(bfoSteps) / sizeof(bfoSteps[0]);
