#include "Arduino.h"
#include "Encoders.h"

#define RH_ENCODER_A 20
#define RH_ENCODER_B 19
#define LH_ENCODER_A 35
#define LH_ENCODER_B 47


volatile long left_count = 0;
volatile long right_count = 0;

void IRAM_ATTR leftEncoderEvent(){
  if(digitalRead(LH_ENCODER_A) == HIGH)
    if(digitalRead(LH_ENCODER_B) == LOW) left_count--;
    else left_count++;
  else
    if(digitalRead(LH_ENCODER_B) == LOW) left_count++;
    else left_count--;
}

void IRAM_ATTR rightEncoderEvent(){
  if(digitalRead(RH_ENCODER_A) == HIGH)
    if(digitalRead(RH_ENCODER_B) == LOW) right_count++;
    else right_count--;
  else
    if(digitalRead(RH_ENCODER_B) == LOW) right_count--;
    else right_count++;
}

Encoders::Encoders() {
	pinMode(RH_ENCODER_A, INPUT);
	pinMode(RH_ENCODER_B, INPUT);
	pinMode(LH_ENCODER_A, INPUT);
	pinMode(LH_ENCODER_B, INPUT);
}

void Encoders::setup() {
  attachInterrupt(digitalPinToInterrupt(LH_ENCODER_A), leftEncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RH_ENCODER_A), rightEncoderEvent, CHANGE);
}

volatile long Encoders::get_left_count() {
    return left_count;
}

volatile long Encoders::get_right_count(){
    return right_count;
}