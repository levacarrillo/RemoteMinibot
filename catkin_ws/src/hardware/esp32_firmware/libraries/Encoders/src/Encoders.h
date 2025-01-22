#ifndef Encoders_h
#define Encoders_h

#include "Arduino.h"

class Encoders {
    public:
            Encoders();
            void setup();
            volatile long get_left_count();
            volatile long get_right_count();
};

#endif