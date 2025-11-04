#include <Arduino.h>

// === Pin Assignments ===
struct ThreeStateSwitch {
  int pinA;
  int pinB;
  char actionA;  // prixnted when pinA active
  char actionB;  // printed when pinB active
  int lastState; // previous state (-1, 0, +1)
};

ThreeStateSwitch switches[] = {
  {4, 5, 'A', 'D', 0}, // left/right
  {6, 7, 'W', 'S', 0}, // fwd/back
  {15, 16, 'N', 'M', 0}, // cw/ccw (heading)
  {17, 18, '+', '-', 0} // up/down (altitude)
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

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nESP32-S3 Three-State Switch Monitor");

  for (int i = 0; i < numSwitches; i++) {
    pinMode(switches[i].pinA, INPUT_PULLUP);
    pinMode(switches[i].pinB, INPUT_PULLUP);
  }
}

void loop() {
  // Serial.print('.'); // heartbeat

  for (int i = 0; i < numSwitches; i++) {
    int state = read3StateSwitch(switches[i].pinA, switches[i].pinB);
    switches[i].lastState = state;
    if (state == +1) {
      Serial.print(switches[i].actionA);
    } else if (state == -1) {
      Serial.print(switches[i].actionB);
    }
  }

  delay(50); // debounce interval
}