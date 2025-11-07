#include <Arduino.h>

// === Pin Assignments ===
struct ThreeStateSwitch {
  int pinA;
  int pinB;
  char actionA;  // prixnted when pinA active
  char actionB;  // printed when pinB active
  int lastState; // previous state (-1, 0, +1)
};

int read3StateSwitch(int pinA, int pinB);

void setup();

void loop();