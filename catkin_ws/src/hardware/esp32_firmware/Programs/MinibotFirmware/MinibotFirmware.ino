#include <ros.h>
#include <Sensors.h>
#include <Encoders.h>
#include <PID.h>
#include <std_msgs/Float32MultiArray.h>



#define BAUD 115200

ros::NodeHandle nh;

PID motors;
Sensors sensors;
Encoders encoders;


std_msgs::Float32MultiArray sensors_msg;

void motorsSpeedCallback(const std_msgs::Float32MultiArray& msg);

ros::Publisher sensorsPub("/hardware/sensors", &sensors_msg);
ros::Subscriber<std_msgs::Float32MultiArray> subMotorsSpeed("/hardware/speed_motors", motorsSpeedCallback);

void publish_sensors() {
  sensors_msg.data_length = 21;
  
  float data[20];
  float* speeds = motors.getCurrVelocities();
  sensors.read();

  int* light_sensors_array = sensors.get_lights_status();
  for (int i=0; i<8; i++) {
    data[i] = light_sensors_array[i];
  }
  for (int i=8; i<12; i++) {
    data[i] = speeds[i-8];
  }
  
  bool* line_sensors = sensors.get_line_status();
  for (int i=12; i<15; i++) {
    data[i] = line_sensors[i-12];
  }

  data[15] = encoders.get_left_count();
  data[16] = encoders.get_right_count();

  data[17] = sensors.get_battery_status();

  int* sharp_sensors_array = sensors.get_sharps_status();
  for (int i=18; i<21; i++) {
    data[i] = sharp_sensors_array[i-18];
  }

  sensors_msg.data = data;
  sensorsPub.publish(&sensors_msg);
}

void motorsSpeedCallback(const std_msgs::Float32MultiArray& msg){
  motors.setSpeeds(msg.data[0], msg.data[1]);
}


void setup() {
  delay(2000);
  nh.getHardware()->setBaud(BAUD);
  nh.initNode();
  encoders.setup();

  float left_pid[3] = {0.055, 0.7, 0.05}, right_pid[3] = {0.05, 0., 0.0};
  motors.setLeftPID(left_pid);
  motors.setRightPID(right_pid);

  nh.advertise(sensorsPub);
  nh.subscribe(subMotorsSpeed);
}

void loop() {
  motors.setEncodersCount(encoders.get_left_count(), encoders.get_right_count());
  publish_sensors();
  nh.spinOnce();
  delay(10);
}
