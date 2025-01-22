#include "Motors.h"

Motors motors;

void setup() {
}

void loop() {
    motors.move(0, 0);
    motors.move(255, 255);
    delay(3000);
    motors.move(0, 0);
    delay(3000);
    motors.move(90, 90);
    delay(3000);
    motors.move(0, 0);
    delay(3000);
    motors.move(-255, -255);
    delay(3000);
    motors.move(0, 0);
    delay(3000);
    motors.move(-90, -90);
    delay(3000);
    motors.move(0, 0);
    delay(3000);
    motors.move(255, -255);
    delay(3000);
    motors.move(0, 0);
    delay(3000);
    motors.move(90, -90);
    delay(3000);
    motors.move(0, 0);
    delay(3000);
    motors.move(-255, 255);
    delay(3000);
    motors.move(0, 0);
    delay(3000);
    motors.move(-90, 90);
    delay(3000);
    motors.move(0, 0);
    delay(3000);
}
