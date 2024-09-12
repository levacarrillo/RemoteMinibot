#ifndef Blink_h
#define Blink_h

#include "Arduino.h"

class Blink {
    private: 
	    int _led;
    public:
	    Blink(int led);
	    void myFunction(int blinkRate);

};
#endif
