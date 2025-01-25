#ifndef Sensors_h
#define Sensors_h

#include "Arduino.h"

class Sensors {
    private:
        int ldr[8];
	    int sharp[8];
	    bool line[3];
	    int battery;
		bool stop_button;

    public: 
	    Sensors();
	    void read();
        int get_battery_status();
		bool* get_line_status();
	    int* get_lights_status();
	    int* get_sharps_status();
		bool get_stop_button_status();
};
#endif
