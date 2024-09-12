#include "Arduino.h"
#include "Motors.h"

#define MOTOR_LEFT0  4
#define MOTOR_LEFT1 10
#define MOTOR_RIGHT0  6
#define MOTOR_RIGHT1  7

#define ENABLE_LEFT   5
#define ENABLE_RIGHT  11

Motors::Motors() {
	pinMode(MOTOR_LEFT0, OUTPUT);
	pinMode(MOTOR_LEFT1, OUTPUT);
	pinMode(ENABLE_LEFT,  OUTPUT);
	pinMode(MOTOR_RIGHT0, OUTPUT);
	pinMode(MOTOR_RIGHT1, OUTPUT);
	pinMode(ENABLE_RIGHT,  OUTPUT);
}

void Motors::move(bool left_foward, int left_pwm, bool right_foward, int right_pwm) {
	digitalWrite(MOTOR_LEFT0, !left_foward);
	digitalWrite(MOTOR_LEFT1,  left_foward);
	analogWrite(ENABLE_LEFT,   left_pwm);
        
	digitalWrite(MOTOR_RIGHT0,  right_foward);
	digitalWrite(MOTOR_RIGHT1, !right_foward);
	analogWrite(ENABLE_RIGHT,   right_pwm);
}