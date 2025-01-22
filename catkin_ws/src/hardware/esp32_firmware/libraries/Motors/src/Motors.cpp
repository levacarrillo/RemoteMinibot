#include "Arduino.h"
#include "Motors.h"
#include "driver/ledc.h"

#define MOTOR_LEFT_IN1 40
#define MOTOR_LEFT_IN2 41
#define MOTOR_RIGHT_IN3 38
#define MOTOR_RIGHT_IN4 21

#define PWM_FREQ 5000
#define PWM_RESOLUTION 8


Motors::Motors() {
	ledcAttach(MOTOR_LEFT_IN1, PWM_FREQ, PWM_RESOLUTION);
	ledcAttach(MOTOR_LEFT_IN2, PWM_FREQ, PWM_RESOLUTION);
	ledcAttach(MOTOR_RIGHT_IN3, PWM_FREQ, PWM_RESOLUTION);
	ledcAttach(MOTOR_RIGHT_IN4, PWM_FREQ, PWM_RESOLUTION);
}

void Motors::move(int left_pwm, int right_pwm) {
	if (left_pwm < 0) {
		ledcWrite(MOTOR_LEFT_IN1, -left_pwm);
		ledcWrite(MOTOR_LEFT_IN2, 0);
	} else if (left_pwm > 0){
		ledcWrite(MOTOR_LEFT_IN1, 0);
		ledcWrite(MOTOR_LEFT_IN2, left_pwm);
	} else {
		ledcWrite(MOTOR_LEFT_IN1, 0);
		ledcWrite(MOTOR_LEFT_IN2, 0);
	}
    if (right_pwm < 0) {
		ledcWrite(MOTOR_RIGHT_IN3, -right_pwm);
		ledcWrite(MOTOR_RIGHT_IN4, 0);
	} else if (right_pwm > 0) {
		ledcWrite(MOTOR_RIGHT_IN3, 0);
		ledcWrite(MOTOR_RIGHT_IN4, right_pwm);
	} else {
		ledcWrite(MOTOR_RIGHT_IN3, 0);
		ledcWrite(MOTOR_RIGHT_IN4, 0);
	}
}