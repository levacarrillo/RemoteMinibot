#ifndef PID_h
#define PID_h

#include "Arduino.h"

class PID {
    private:
        float LP, LI, LD;
        float RP, RI, RD;
    public:
        PID();
        float* getLeftPID();
        float* getRightPID();
        void setLeftPID(float* PID);
        void setRightPID(float* PID);
        void setSpeeds(float left_speed, float right_speed);
        void setEncodersCount(volatile long left_count, volatile long right_count);
};

#endif
