#include "rds_helpers.h"
#include <string.h>

void resetRdsState(RdsState &state) {
  state.synced = false;
  state.station[0] = '\0';
  state.text[0] = '\0';
  state.scrollIndex = 0;
  state.lastUpdate = 0;
  state.lastScroll = 0;
}

void sanitizeRdsText(const char *input, char *output, size_t outputSize) {
  if (!input || outputSize == 0) {
    return;
  }
  size_t outIndex = 0;
  for (size_t i = 0; input[i] != '\0' && outIndex < outputSize - 1; i++) {
    char c = input[i];
    if (c < 32) {
      c = ' ';
    }
    output[outIndex++] = c;
  }
  output[outIndex] = '\0';

  size_t start = 0;
  while (output[start] == ' ' && output[start] != '\0') {
    start++;
  }
  if (start > 0) {
    memmove(output, output + start, outIndex - start + 1);
  }
  size_t len = strlen(output);
  while (len > 0 && output[len - 1] == ' ') {
    output[len - 1] = '\0';
    len--;
  }
}

void buildRdsScrollLine(const RdsState &state, char *lineBuffer, size_t lineSize) {
  if (lineSize == 0) {
    return;
  }
  lineBuffer[0] = '\0';
  size_t textLen = strlen(state.text);
  if (textLen == 0) {
    return;
  }
  if (textLen <= 20) {
    snprintf(lineBuffer, lineSize, "%s", state.text);
    return;
  }
  for (uint8_t i = 0; i < 20; i++) {
    lineBuffer[i] = state.text[state.scrollIndex + i];
  }
  lineBuffer[20] = '\0';
}

bool refreshRdsStatus(SI4735 &radio, RdsState &state, bool force) {
  uint32_t now = millis();
  if (!force && (now - state.lastUpdate) < RDS_UPDATE_MS) {
    return false;
  }
  state.lastUpdate = now;

  radio.getRdsStatus();
  if (!radio.getRdsReceived()) {
    return false;
  }
  if (!radio.getRdsSync() || radio.getNumRdsFifoUsed() == 0) {
    if (state.synced) {
      state.synced = false;
      return true;
    }
    return false;
  }

  bool updated = false;
  state.synced = true;

  char *stationName = radio.getRdsStationName();
  if (stationName != nullptr) {
    char stationBuffer[9];
    sanitizeRdsText(stationName, stationBuffer, sizeof(stationBuffer));
    if (stationBuffer[0] != '\0' && strcmp(state.station, stationBuffer) != 0) {
      snprintf(state.station, sizeof(state.station), "%s", stationBuffer);
      updated = true;
    }
  }

  char *programInfo = radio.getRdsProgramInformation();
  if (programInfo != nullptr) {
    char textBuffer[65];
    sanitizeRdsText(programInfo, textBuffer, sizeof(textBuffer));
    if (textBuffer[0] != '\0' && strcmp(state.text, textBuffer) != 0) {
      snprintf(state.text, sizeof(state.text), "%s", textBuffer);
      state.scrollIndex = 0;
      updated = true;
    }
  }

  return updated;
}

bool updateRdsScroll(RdsState &state) {
  if (state.text[0] == '\0') {
    return false;
  }
  uint32_t now = millis();
  if ((now - state.lastScroll) < RDS_SCROLL_MS) {
    return false;
  }
  state.lastScroll = now;
  size_t textLen = strlen(state.text);
  if (textLen <= 20) {
    return false;
  }
  state.scrollIndex = state.scrollIndex + RDS_SCROLL_STEP;
  if (state.scrollIndex > (textLen - 20)) {
    state.scrollIndex = 0;
  }
  return true;
}
