// Rotary encoder library for Arduino (header-only).
#ifndef rotary_h
#define rotary_h

#include "Arduino.h"

// Enable this to emit codes twice per step
// #define HALF_STEP

#define ENABLE_PULLUPS  // Enable weak pullups

// Values returned by 'process'
#define DIR_NONE 0x0    // No complete step yet
#define DIR_CW   0x10   // Clockwise step
#define DIR_CCW  0x20   // Anti-clockwise step

class Rotary
{
  public:
    Rotary(char, char);
    // Process pin(s)
    unsigned char process();
  private:
    unsigned char state;
    unsigned char pin1;
    unsigned char pin2;
};

// Implementation
#define R_START       0x0
#ifdef HALF_STEP
// Use the half-step state table (emits a code at 00 and 11)
#define R_CCW_BEGIN   0x1
#define R_CW_BEGIN    0x2
#define R_START_M     0x3
#define R_CW_BEGIN_M  0x4
#define R_CCW_BEGIN_M 0x5
static const unsigned char ttable[6][4] = {
  // R_START (00)
  {R_START_M, R_CW_BEGIN, R_CCW_BEGIN, R_START},
  // R_CCW_BEGIN
  {R_START_M | DIR_CCW, R_START, R_CCW_BEGIN, R_START},
  // R_CW_BEGIN
  {R_START_M | DIR_CW, R_CW_BEGIN, R_START, R_START},
  // R_START_M (11)
  {R_START_M, R_CCW_BEGIN_M, R_CW_BEGIN_M, R_START},
  // R_CW_BEGIN_M
  {R_START_M, R_START_M, R_CW_BEGIN_M, R_START | DIR_CW},
  // R_CCW_BEGIN_M
  {R_START_M, R_CCW_BEGIN_M, R_START_M, R_START | DIR_CCW},
};
#else
// Use the full-step state table (emits a code at 00 only)
#define R_CW_FINAL  0x1
#define R_CW_BEGIN  0x2
#define R_CW_NEXT   0x3
#define R_CCW_BEGIN 0x4
#define R_CCW_FINAL 0x5
#define R_CCW_NEXT  0x6

static const unsigned char ttable[7][4] = {
  // R_START
  {R_START, R_CW_BEGIN, R_CCW_BEGIN, R_START},
  // R_CW_FINAL
  {R_CW_NEXT, R_START, R_CW_FINAL, R_START | DIR_CW},
  // R_CW_BEGIN
  {R_CW_NEXT, R_CW_BEGIN, R_START, R_START},
  // R_CW_NEXT
  {R_CW_NEXT, R_CW_BEGIN, R_CW_FINAL, R_START},
  // R_CCW_BEGIN
  {R_CCW_NEXT, R_START, R_CCW_BEGIN, R_START},
  // R_CCW_FINAL
  {R_CCW_NEXT, R_CCW_FINAL, R_START, R_START | DIR_CCW},
  // R_CCW_NEXT
  {R_CCW_NEXT, R_CCW_FINAL, R_CCW_BEGIN, R_START},
};
#endif

inline Rotary::Rotary(char _pin1, char _pin2) {
  pin1 = _pin1;
  pin2 = _pin2;
  pinMode(pin1, INPUT);
  pinMode(pin2, INPUT);
#ifdef ENABLE_PULLUPS
  digitalWrite(pin1, HIGH);
  digitalWrite(pin2, HIGH);
#endif
  state = R_START;
}

inline unsigned char Rotary::process() {
  unsigned char pinstate = (digitalRead(pin2) << 1) | digitalRead(pin1);
  state = ttable[state & 0xf][pinstate];
  return state & 0x30;
}

#endif
