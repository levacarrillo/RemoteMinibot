// SIMPLE BLINK FOR ESP32 S3 USING A CUSTOM LIBRARY
#include "Blink.h"

Blink blink(48);
int blinkRate = 2000;

void setup() {
} 

void loop() {
  blink.myFunction(blinkRate);
}
