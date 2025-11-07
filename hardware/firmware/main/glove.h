#pragma once

#include <cstdint>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// === Pin Assignments ===
struct ThreeStateSwitch {
  gpio_num_t pinA;   // GPIO pin for A
  gpio_num_t pinB;   // GPIO pin for B
  char actionA;      // printed when pinA active
  char actionB;      // printed when pinB active
  int8_t lastState;  // previous state (-1, 0, +1)
};

// Read a three-state switch connected to two GPIOs.
// Returns -1, 0 or +1.
int8_t read3StateSwitch(gpio_num_t pinA, gpio_num_t pinB);

// Initialize glove hardware (configure GPIOs, etc.).
void glove_init();

// Optional FreeRTOS task entry that can run glove processing in a loop.
// Use xTaskCreatePinnedToCore/glove_task(...) or call glove_init() from app_main.
void glove_task(void *arg);