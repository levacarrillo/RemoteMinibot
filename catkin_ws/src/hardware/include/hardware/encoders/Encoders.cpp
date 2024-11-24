#include "Arduino.h"
#include "Encoders.h"

#define RH_ENCODER_A 2
#define RH_ENCODER_B 4
#define LH_ENCODER_A 3
#define LH_ENCODER_B 5

volatile long Encoders::left_count = 0;
volatile long Encoders::right_count = 0;

void Encoders::leftEncoderEvent(){
  if(digitalRead(LH_ENCODER_A) == HIGH)
    if(digitalRead(LH_ENCODER_B) == LOW) Encoders::left_count--;
    else Encoders::left_count++;
  else
    if(digitalRead(LH_ENCODER_B) == LOW) Encoders::left_count++;
    else Encoders::left_count--;
}

void Encoders::rightEncoderEvent(){
  if(digitalRead(RH_ENCODER_A) == HIGH)
    if(digitalRead(RH_ENCODER_B) == LOW) Encoders::right_count++;
    else Encoders::right_count--;
  else
    if(digitalRead(RH_ENCODER_B) == LOW) Encoders::right_count--;
    else Encoders::right_count++;
}


Encoders::Encoders() {
  left_count  = 0;
  right_count = 0;
	pinMode(RH_ENCODER_A, INPUT);
	pinMode(RH_ENCODER_B, INPUT);
	pinMode(LH_ENCODER_A, INPUT);
	pinMode(LH_ENCODER_B, INPUT);

	attachInterrupt(0, rightEncoderEvent, CHANGE);
	attachInterrupt(1, leftEncoderEvent, CHANGE);
}

volatile long Encoders::get_left_count() {
    return left_count;
}

volatile long Encoders::get_right_count(){
    return right_count;
}