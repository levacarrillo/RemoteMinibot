#include "Sensors.h"

Sensors sensors;

int* light_sensor;
int* sharp_sensor;
int battery_sensor;
bool* line_sensor;

void setup() {
    Serial.begin(9600);
}

void loop() {
    sensors.read();

	light_sensor = sensors.get_lights_status();
	Serial.print("lights: [");
        Serial.print(light_sensor[0]);
	for(int i=1; i<8; i++)
	{
	  Serial.print(", ");
          Serial.print(light_sensor[i]);
	}
	Serial.print("]\t");//*/

	sharp_sensor = sensors.get_sharps_status();
	Serial.print("sharps: [");
        Serial.print(sharp_sensor[0]);
	for(int i=1; i<8; i++)
	{
	  Serial.print(", ");
          Serial.print(sharp_sensor[i]);
	}
	Serial.print("]\t");//*/

	battery_sensor = sensors.get_battery_status();
	Serial.print("battery: [");
	Serial.print(battery_sensor);
	Serial.print("]\t");
	
	line_sensor = sensors.get_line_status();
	Serial.print("line: [");
	Serial.print(line_sensor[0]);
	Serial.print(", ");
	Serial.print(line_sensor[1]);
    Serial.print(", ");
	Serial.print(line_sensor[2]);
	Serial.println("]\t");
	
	delay(200);	
}
