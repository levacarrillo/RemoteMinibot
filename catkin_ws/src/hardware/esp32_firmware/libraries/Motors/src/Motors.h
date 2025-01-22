#ifndef Motors_h
#define Motors_h

#include "Arduino.h"

class Motors {
    public:
        Motors();
        void move(int left_pwm, int right_pwm);
};

#endif