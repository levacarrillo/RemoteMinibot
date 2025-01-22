#include "Encoders.h"

Encoders encoders;

volatile long l_count;
volatile long r_count;

void setup() {
    Serial.begin(9600);
    encoders.setup();
}

void loop() {
    l_count  = encoders.get_left_count();
    r_count = encoders.get_right_count();
    Serial.print("left count: ");
    Serial.print(l_count);
    Serial.print("\t");
        
    Serial.print("right count: ");
    Serial.print(r_count);
    Serial.println("\t");
}
