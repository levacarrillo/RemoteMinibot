#include "Arduino.h"
#include "PID.h"
#include "../motors/Motors.h"


Motors* _motors;

int LEFT  = 0;
int RIGHT = 1;

float goal_speed[2];
volatile long rel_left_count  = 0;
volatile long rel_right_count = 0;

float wheel_diameter = (43.38) / 1000;

unsigned int pulses_per_turn = 600;
unsigned int previous_time = millis();

double sampling_time = 40;
double sampling_timeSec = sampling_time / 1000.0;

double last_left_speed_error  = 0;
double last_right_speed_error = 0;

PID::PID() {}

float* calculateCurrentSpeeds(volatile long left_count, volatile long right_count, double elapsed_time) {
    static float speed[2];
    if (elapsed_time >= sampling_time) {
        rel_left_count  = left_count  - rel_left_count;
        rel_right_count = right_count - rel_right_count;
        float curr_left_speed  = M_PI * wheel_diameter * fabs(rel_left_count)  / pulses_per_turn * 1000.0 / elapsed_time;
        float curr_right_speed = M_PI * wheel_diameter * fabs(rel_right_count) / pulses_per_turn * 1000.0 / elapsed_time;
        
        rel_left_count  = left_count;
        rel_right_count = right_count;
        speed[LEFT]  = curr_left_speed;
        speed[RIGHT] = curr_right_speed;
    } 

    return speed;
}

int* speedsToPwm(double left_pid_output, double right_pid_output) {
    int left_pwm  = map(left_pid_output  * 100, 0, 20, 0, 255);
    int right_pwm = map(right_pid_output * 100, 0, 20, 0, 255);
    
    if(left_pwm  < 0)  left_pwm = 0;
    if(right_pwm < 0) right_pwm = 0;  
    if(left_pwm  > 255)  left_pwm = 255;
    if(right_pwm > 255) right_pwm = 255;

    static int pwm[2];
    pwm[LEFT]  = left_pwm;
    pwm[RIGHT] = right_pwm;
    return pwm;
}

void  PID::setLeftPID(float* LPID) {
    LP = LPID[0];
    LI = LPID[1];
    LD = LPID[2];
}

void  PID::setRightPID(float* RPID) {
    RP = RPID[0];
    RI = RPID[1];
    RD = RPID[2];
}

float* PID::getLeftPID() {
    static float pid[3];
    pid[0] = LP;
    pid[1] = LI;
    pid[2] = LD;
    return pid;
}

float* PID::getRightPID() {
    static float pid[3];
    pid[0] = RP;
    pid[1] = RI;
    pid[2] = RD;
    return pid;
}

void PID::setSpeeds(float goal_left_speed, float goal_right_speed) {
    goal_speed[LEFT]  = goal_left_speed;
    goal_speed[RIGHT] = goal_right_speed;
}

void PID::setEncodersCount(volatile long left_count, volatile long right_count) {
    
    unsigned int current_time = millis();
    bool left_foward = true, right_foward = true;
    double elapsed_time = (double)(current_time - previous_time);
    float* current_speed = calculateCurrentSpeeds(left_count, right_count, elapsed_time);

    double left_speed_error  = fabs(goal_speed[LEFT])  - current_speed[LEFT];
    double right_speed_error = fabs(goal_speed[RIGHT]) - current_speed[RIGHT];

    float left_speed_error_area  = left_speed_error  * elapsed_time;
    float right_speed_error_area = right_speed_error * elapsed_time;

    float left_speed_error_gradient  = (left_speed_error  - last_left_speed_error)  / sampling_time;
    float right_speed_error_gradient = (right_speed_error - last_right_speed_error) / sampling_time;

    double left_pid_output  = LP * left_speed_error +
                       LI * left_speed_error_area +
                       LD * left_speed_error_gradient;

    double right_pid_output = RP * right_speed_error +
                       RI * right_speed_error_area +
                       RD * right_speed_error_gradient;

    int* pwm = speedsToPwm(left_pid_output, right_pid_output);
    
    previous_time = current_time;
    last_left_speed_error  = left_speed_error;
    last_right_speed_error = right_speed_error;
    
    if (goal_speed[LEFT]  < 0) left_foward  = false;
    if (goal_speed[RIGHT] < 0) right_foward = false;
    
    _motors->move(left_foward, pwm[LEFT], right_foward, pwm[RIGHT]);
}