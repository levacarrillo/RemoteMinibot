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
        float* getCurrVelocities();
        void setLeftPID(float* LEFT_PID);
        void setRightPID(float* RIGHT_PID);
        void setSpeeds(float left_speed, float right_speed);
        void setEncodersCount(volatile long left_count, volatile long right_count);
};

#endif
