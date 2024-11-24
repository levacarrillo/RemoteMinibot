#include "Arduino.h"
#include "Motors.h"

#define MOTOR_LEFT0   8
#define MOTOR_LEFT1   9
#define MOTOR_RIGHT0  10
#define MOTOR_RIGHT1  11


Motors::Motors() {
	pinMode(MOTOR_LEFT0, OUTPUT);
	pinMode(MOTOR_LEFT1, OUTPUT);
	pinMode(MOTOR_RIGHT0, OUTPUT);
	pinMode(MOTOR_RIGHT1, OUTPUT);
}

void Motors::move(int left_pwm, int right_pwm) {
	if (left_pwm < 0) {
		analogWrite(MOTOR_LEFT0,  -left_pwm);		
		digitalWrite(MOTOR_LEFT1, 0);
	} else {
		digitalWrite(MOTOR_LEFT0, 0);
		analogWrite(MOTOR_LEFT1,  left_pwm);
	}
    if (right_pwm < 0) {
		analogWrite(MOTOR_RIGHT0, -right_pwm);
		digitalWrite(MOTOR_RIGHT1,  0);
	} else {
		digitalWrite(MOTOR_RIGHT0,  0);
		analogWrite(MOTOR_RIGHT1, right_pwm);
	}
}