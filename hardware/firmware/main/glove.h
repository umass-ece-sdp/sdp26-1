#pragma once

#include <Arduino.h>

// === Pin Assignments ===
struct ThreeStateSwitch {
  int pinA;
  int pinB;
  char actionA;  // printed when pinA active
  char actionB;  // printed when pinB active
  int lastState; // previous state (-1, 0, +1)
};

// Global switch array
extern ThreeStateSwitch switches[];
extern const int numSwitches;

// Function prototypes
int read3StateSwitch(int pinA, int pinB);
void setup_glove();
void read_glove_inputs(char* output, int maxLen);