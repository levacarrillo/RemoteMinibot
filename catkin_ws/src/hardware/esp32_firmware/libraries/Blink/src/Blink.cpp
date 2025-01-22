#include "Arduino.h"
#include "Blink.h"

Blink::Blink(int led) {
    pinMode(led, OUTPUT);
    _led = led;
}

void Blink::myFunction(int blinkRate) {
    digitalWrite(_led, HIGH);
    delay(blinkRate);
    digitalWrite(_led, LOW);
    delay(blinkRate);
}
