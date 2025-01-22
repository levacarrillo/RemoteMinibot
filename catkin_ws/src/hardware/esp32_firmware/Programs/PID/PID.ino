#include <Encoders.h>
#include <PID.h>

PID motors;
Encoders encoders;

void setup() {
    Serial.begin(9600);
    float left_pid[3] = {0.1, 0.0, 0.0};
    float right_pid[3] = {100.0, 0.0, 0.0};
    
    encoders.setup();
    motors.setLeftPID(left_pid);
    motors.setRightPID(right_pid);
    motors.setSpeeds(0.0, 0.0);
}

void loop() {
    Serial.print("left_output: ");
    Serial.print(encoders.get_left_count());
    Serial.print("\t");

    Serial.print("right_output: ");
    Serial.print(encoders.get_right_count());
    Serial.println("\t");
    motors.setEncodersCount(encoders.get_left_count(), encoders.get_right_count());
    delay(20);
}