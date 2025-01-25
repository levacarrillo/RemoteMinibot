#include "Arduino.h"
#include "Sensors.h"


// LINE SENSORS
#define RIGHT_LINE_SENSOR   2
#define LEFT_LINE_SENSOR    37
#define CENTER_LINE_SENSOR  42 

// STOP BUTTON
#define STOP_BUTTON 39




Sensors::Sensors() {
    pinMode(RIGHT_LINE_SENSOR,  INPUT);
    pinMode(LEFT_LINE_SENSOR,   INPUT);
    pinMode(CENTER_LINE_SENSOR, INPUT);   
    pinMode(STOP_BUTTON, INPUT);
}

void Sensors::read() {
    // LIGHT SENSORS
    ldr[0] = analogRead(18);//
    ldr[1] = analogRead(17);
    ldr[2] = analogRead(16);
    ldr[3] = analogRead(15);
    ldr[4] = analogRead(4);
    ldr[5] = analogRead(5);
    ldr[6] = analogRead(6);
    ldr[7] = analogRead(7);//

    // SHARP SENSORS
    sharp[0] = analogRead(8);
    sharp[1] = analogRead(3);
    sharp[2] = analogRead(9);
    sharp[3] = analogRead(10);
    sharp[4] = analogRead(11);
    sharp[5] = analogRead(12);
    sharp[6] = analogRead(13);
    sharp[7] = analogRead(14);

    // BATTERY SENSOR
    battery = analogRead(1);

    // STOP BUTTON
    stop_button = digitalRead(STOP_BUTTON);


    line[0] = digitalRead(RIGHT_LINE_SENSOR);
    line[1] = digitalRead(LEFT_LINE_SENSOR);
    line[2] = digitalRead(CENTER_LINE_SENSOR); 
}

int Sensors::get_battery_status() {
    return battery;    
}

int* Sensors::get_lights_status() {
    return ldr;    
}

int* Sensors::get_sharps_status() {
    return sharp;
}

bool* Sensors::get_line_status() {
    return line;
}

bool Sensors::get_stop_button_status() {
    return stop_button;
}
