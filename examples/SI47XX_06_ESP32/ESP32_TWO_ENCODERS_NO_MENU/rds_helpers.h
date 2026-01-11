#ifndef ESP32_TWO_ENCODERS_RDS_HELPERS_H
#define ESP32_TWO_ENCODERS_RDS_HELPERS_H

#include <Arduino.h>
#include <SI4735.h>
#include "config.h"

struct RdsState {
  bool synced;
  char station[9];
  char text[65];
  uint8_t scrollIndex;
  uint32_t lastUpdate;
  uint32_t lastScroll;
};

void resetRdsState(RdsState &state);
void sanitizeRdsText(const char *input, char *output, size_t outputSize);
void buildRdsScrollLine(const RdsState &state, char *lineBuffer, size_t lineSize);
bool refreshRdsStatus(SI4735 &radio, RdsState &state, bool force);
bool updateRdsScroll(RdsState &state);

#endif
