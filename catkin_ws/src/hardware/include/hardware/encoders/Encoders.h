#ifndef Encoders_h
#define Encoders_h

#include "Arduino.h"

class Encoders {
    private:
            volatile static long left_count;
            volatile static long right_count;

    public:
            Encoders();
            volatile long get_left_count();
            volatile long get_right_count();
            static void leftEncoderEvent();
            static void rightEncoderEvent();
};

#endif