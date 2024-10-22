#ifndef Motors_h
#define Motors_h

#include "Arduino.h"

class Motors {
    public:
        Motors();
        void move(bool left_foward, int left_pwm, bool right_foward, int right_pwm);
};
#endif