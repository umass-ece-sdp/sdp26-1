#include "glove.h"
#include <stdio.h>
#include "esp_log.h"

static const char *TAG = "glove";

ThreeStateSwitch switches[] = {
  {(gpio_num_t)4,  (gpio_num_t)5,  'A', 'D', 0}, // left/right
  {(gpio_num_t)6,  (gpio_num_t)7,  'W', 'S', 0}, // fwd/back
  {(gpio_num_t)15, (gpio_num_t)16, 'N', 'M', 0}, // cw/ccw (heading)
  {(gpio_num_t)17, (gpio_num_t)18, '+', '-', 0}  // up/down (altitude)
};

const int numSwitches = sizeof(switches) / sizeof(switches[0]);

// Read 3-state switch: +1, 0, -1
int8_t read3StateSwitch(gpio_num_t pinA, gpio_num_t pinB) {
  // With pull-up enabled, a LOW (0) means switch is active
  bool a = (gpio_get_level(pinA) == 0);
  bool b = (gpio_get_level(pinB) == 0);
  if (a && !b) return +1;
  if (!a && b) return -1;
  return 0;
}

// Configure GPIOs for the switches (enable pull-ups)
void glove_init() {
  ESP_LOGI(TAG, "Initializing glove hardware...");
  for (int i = 0; i < numSwitches; i++) {
    // Ensure pad is selected for GPIO
    gpio_pad_select_gpio(switches[i].pinA);
    gpio_set_direction(switches[i].pinA, GPIO_MODE_INPUT);
    gpio_pullup_en(switches[i].pinA);

    gpio_pad_select_gpio(switches[i].pinB);
    gpio_set_direction(switches[i].pinB, GPIO_MODE_INPUT);
    gpio_pullup_en(switches[i].pinB);
  }
  ESP_LOGI(TAG, "Glove hardware initialized");
}

// FreeRTOS task entry: run the old loop() logic here
void glove_task(void *arg) {
  (void)arg;
  while (true) {
    for (int i = 0; i < numSwitches; i++) {
      int8_t state = read3StateSwitch(switches[i].pinA, switches[i].pinB);
      switches[i].lastState = state;
      if (state == +1) {
        putchar(switches[i].actionA);
      } else if (state == -1) {
        putchar(switches[i].actionB);
      }
    }
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(50)); // debounce interval
  }
}