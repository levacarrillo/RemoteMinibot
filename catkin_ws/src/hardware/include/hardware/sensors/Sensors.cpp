#include "Arduino.h"
#include "Sensors.h"

#define RIGHT_LINE_SENSOR   12
#define LEFT_LINE_SENSOR    13
#define CENTER_LINE_SENSOR  22 

Sensors::Sensors() {
    pinMode(RIGHT_LINE_SENSOR,  INPUT);
    pinMode(LEFT_LINE_SENSOR,   INPUT);
    pinMode(CENTER_LINE_SENSOR, INPUT);   
}

void Sensors::read() {
    for(int i=0; i<8; i++)
        ldr[i] = analogRead(i);

    for(int i=8; i<15; i++)
        sharp[i-8] = analogRead(i);

    battery = analogRead(15);

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

int* Sensors::get_line_status() {
    return line;
}
