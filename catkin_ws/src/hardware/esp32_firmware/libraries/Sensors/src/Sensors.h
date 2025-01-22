#ifndef Sensors_h
#define Sensors_h

#include "Arduino.h"

class Sensors {
    private:
        int ldr[8];
	    int sharp[8];
	    bool line[3];
	    int battery;

    public: 
	    Sensors();
	    void read();
        int get_battery_status();
		bool* get_line_status();
	    int* get_lights_status();
	    int* get_sharps_status();
};
#endif
