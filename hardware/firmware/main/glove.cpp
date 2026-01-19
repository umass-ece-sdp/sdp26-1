#include "glove.h"

ThreeStateSwitch switches[] = {
  {4, 5, '2', '1', 0}, // left/right
  {6, 7, '2', '1', 0}, // fwd/back
  {15, 16, '2', '1', 0}, // cw/ccw (heading)
  {17, 18, '2', '1', 0} // up/down (altitude)
};

const int numSwitches = sizeof(switches) / sizeof(switches[0]);

// Read 3-state switch: +1, 0, -1
int read3StateSwitch(int pinA, int pinB) {
  // With INPUT_PULLUP, LOW means switch is active
  bool a = digitalRead(pinA) == LOW;
  bool b = digitalRead(pinB) == LOW;
  if (a && !b) return +1;
  if (!a && b) return -1;
  return 0;
}

void setup_glove() {
  Serial.println("[GLOVE] Initializing switches...");
  
  for (int i = 0; i < numSwitches; i++) {
    pinMode(switches[i].pinA, INPUT_PULLUP);
    pinMode(switches[i].pinB, INPUT_PULLUP);
  }
  
  Serial.println("[GLOVE] Switches initialized");
}

void read_glove_inputs(char* output, int maxLen) {
  // Read all switches and build a command string
  int idx = 0;
  
  for (int i = 0; i < numSwitches && idx < maxLen - 1; i++) {
    int state = read3StateSwitch(switches[i].pinA, switches[i].pinB);
    switches[i].lastState = state;
    
    if (state == +1) {
      output[idx++] = switches[i].actionA;
    } else if (state == -1) {
      output[idx++] = switches[i].actionB;
    } else {
      output[idx++] = '0'; // neutral state
    }
  }
  
  output[idx] = '\0'; // null terminate
}