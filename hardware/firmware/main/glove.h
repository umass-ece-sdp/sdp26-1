#pragma once

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN     48
#define LED_COUNT   1
#define LED_BRIGHTNESS 35

// Color Definitions

#define COLOR_PINK  0xAA0000
#define COLOR_BLUE   0x00008B
#define COLOR_YELLOW 0xFFAA00
#define COLOR_OFF   0x000000


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
void update_color(uint32_t color);
void blink_light(uint32_t color, int interval_ms);