#include "Arduino.h"
#include "PID.h"
#include "Motors.h"


Motors _motors;

int LEFT  = 0, RIGHT = 1;

float goal_speed[2];
float curr_speed[2];
float goal_rpm[2];
float curr_rpm[2];

volatile long rel_left_count  = 0;
volatile long rel_right_count = 0;

int max_rpm = 340;
float wheel_diameter = (43.38) / 1000;

unsigned int pulses_per_turn = 600;
unsigned int previous_time = millis();

double sampling_time = 40;
double sampling_timeSec = sampling_time / 1000.0;

double last_left_pid_output = 0.0;
double last_right_pid_output = 0.0;

double left_speed_error1 = 0.0;
double left_speed_error2 = 0.0;
double right_speed_error1 = 0.0;
double right_speed_error2 = 0.0;


PID::PID() {}

void  PID::setLeftPID(float* LEFT_PID) {
    LP = LEFT_PID[0];
    LI = LEFT_PID[1];
    LD = LEFT_PID[2];
}

void  PID::setRightPID(float* RIGHT_PID) {
    RP = RIGHT_PID[0];
    RI = RIGHT_PID[1];
    RD = RIGHT_PID[2];
}

float* PID::getLeftPID() {
    float* LEFT_PID = new float[3];
    LEFT_PID[0] = LP;
    LEFT_PID[1] = LI;
    LEFT_PID[2] = LD;
    return LEFT_PID;
}

float* PID::getRightPID() {
    float* RIGHT_PID = new float[3];
    RIGHT_PID[0] = RP;
    RIGHT_PID[1] = RI;
    RIGHT_PID[2] = RD;
    return RIGHT_PID;
}

void PID::setSpeeds(float goal_left_speed, float goal_right_speed) {
    goal_speed[LEFT]  = goal_left_speed;
    goal_speed[RIGHT] = goal_right_speed;
    goal_rpm[LEFT]  = (goal_left_speed  * 60) / (M_PI * wheel_diameter);
    goal_rpm[RIGHT] = (goal_right_speed * 60) / (M_PI * wheel_diameter);
}

float* PID::getCurrVelocities() {
    static float vel[4];
    vel[0] = goal_speed[LEFT];
    vel[1] = curr_speed[LEFT];
    vel[2] = goal_speed[RIGHT];
    vel[3] = curr_speed[RIGHT];
    // vel[0] = goal_speed[LEFT];
    // vel[1] = curr_speed[LEFT];
    // vel[2] = goal_rpm[LEFT];
    // vel[3] = curr_rpm[LEFT];
    // vel[4] = goal_speed[RIGHT];
    // vel[5] = curr_speed[RIGHT];
    // vel[6] = goal_rpm[RIGHT];
    // vel[7] = curr_rpm[RIGHT];
    return vel;
}

int* speedsToPwm(double left_pid_output, double right_pid_output) {
    int left_pwm  = map(10 * left_pid_output,  0, 10 * max_rpm, 30, 255);
    int right_pwm = map(10 * right_pid_output, 0, 10 * max_rpm, 30, 255);
    
    if(left_pwm  < 0)  left_pwm = 0;
    if(right_pwm < 0) right_pwm = 0;

    if(left_pwm  > 255)  left_pwm = 255;
    if(right_pwm > 255) right_pwm = 255;

    static int pwm[2];
    pwm[LEFT]  = left_pwm;
    pwm[RIGHT] = right_pwm;

    if (goal_speed[LEFT] < 0) {
        pwm[LEFT] = -left_pwm;
    }
    if (goal_speed[RIGHT] < 0) {
        pwm[RIGHT] = -right_pwm;
    }

    return pwm;
}

void calculateCurrentSpeeds(volatile long left_count, volatile long right_count, double elapsed_time) {
    if (elapsed_time >= sampling_time) {
        rel_left_count  = left_count  - rel_left_count;
        rel_right_count = right_count - rel_right_count;

        curr_rpm[LEFT]  = 60000 * fabs(rel_left_count)  / (pulses_per_turn * elapsed_time);
        curr_rpm[RIGHT] = 60000 * fabs(rel_right_count) / (pulses_per_turn * elapsed_time);
        
        curr_speed[LEFT]  = M_PI * wheel_diameter * fabs(rel_left_count)  / pulses_per_turn * 1000.0 / elapsed_time;
        curr_speed[RIGHT] = M_PI * wheel_diameter * fabs(rel_right_count) / pulses_per_turn * 1000.0 / elapsed_time;

        if (goal_speed[LEFT] < 0)
            curr_speed[LEFT]  = - curr_speed[LEFT];
        if (goal_speed[RIGHT] < 0)
            curr_speed[RIGHT] = - curr_speed[RIGHT];
        
        rel_left_count  = left_count;
        rel_right_count = right_count;
        previous_time = elapsed_time + previous_time;
    }
}

void PID::setEncodersCount(volatile long left_count, volatile long right_count) {
    unsigned int current_time = millis();
    double elapsed_time = (double)(current_time - previous_time);
    calculateCurrentSpeeds(left_count, right_count, elapsed_time);

    double left_speed_error  = fabs(goal_rpm[LEFT])  - curr_rpm[LEFT];
    double right_speed_error = fabs(goal_rpm[RIGHT]) - curr_rpm[RIGHT];

    double left_pid_output  =  last_left_pid_output + 
                               (LP + LD/sampling_time) * left_speed_error +
                               (-LP + LI * elapsed_time - 2*LD/elapsed_time)*left_speed_error1 +
                               (LD/sampling_time) * left_speed_error2;

    last_left_pid_output = left_pid_output;
    left_speed_error2 = left_speed_error1;
    left_speed_error1 = left_speed_error2;
    
    double right_pid_output  =  last_right_pid_output + 
                               (RP + RD/sampling_time) * right_speed_error +
                               (-RP + RI * elapsed_time - 2 * RD / elapsed_time) * right_speed_error1 +
                               (RD / sampling_time) * right_speed_error2;

    last_right_pid_output = right_pid_output;
    right_speed_error2 = right_speed_error1;
    right_speed_error1 = right_speed_error2;

    int* pwm = speedsToPwm(left_pid_output, right_pid_output);


    if (goal_rpm[LEFT] == 0) {
        pwm[LEFT] = 0;
    }
    if (goal_rpm[RIGHT] == 0) {
        pwm[RIGHT] = 0;
    }
    
    _motors.move(pwm[LEFT], pwm[RIGHT]);
}
