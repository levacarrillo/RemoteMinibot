#include <ros.h>
#include <Sensors.h>
#include <Encoders.h>
#include <PID.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

#define BAUD 200000

ros::NodeHandle nh;

PID motors;
Sensors sensors;
Encoders encoders;


std_msgs::Int16 batt_perc_msg;
std_msgs::Int32MultiArray encoders_msg;
std_msgs::Int16MultiArray line_sensors_msg;
std_msgs::Int16MultiArray light_sensors_msg;
std_msgs::Int16MultiArray sharp_sensors_msg;

void motorsSpeedCallback(const std_msgs::Float32MultiArray& msg);

ros::Publisher battPercPub("/battery_data", &batt_perc_msg);
ros::Publisher encodersPub("/encoders_data", &encoders_msg);
ros::Publisher lineSensorsPub("/line_sensors", &line_sensors_msg);
ros::Publisher lightSensorsPub("/light_sensors", &light_sensors_msg);
ros::Publisher sharpSensorsPub("/sharp_sensors", &sharp_sensors_msg);
ros::Subscriber<std_msgs::Float32MultiArray> subMotorsSpeed("/speed_motors", motorsSpeedCallback);

printPIDVars(double goal_left_speed, float goal_right_speed) {
  float* pid = motors.getLeftPID();
  char P[8], I[8], D[8], left_speed[8], right_speed[8], log_msg[50];
  nh.logwarn("----------------------------------------------------------------------");
  nh.logwarn("LEFT PID VARIABLES");
  dtostrf(pid[0], 6, 2, P);
  dtostrf(pid[1], 6, 2, I);
  dtostrf(pid[2], 6, 2, D);
  sprintf(log_msg, "P.->%s I.->%s D.->%s", P, I, D);
  nh.logwarn(log_msg);
  pid = motors.getRightPID();
  nh.logwarn("RIGHT PID VARIABLES");
  dtostrf(pid[0], 6, 2, P);
  dtostrf(pid[1], 6, 2, I);
  dtostrf(pid[2], 6, 2, D);
  sprintf(log_msg, "P.->%s I.->%s D.->%s", P, I, D);
  nh.logwarn(log_msg);
  dtostrf(goal_left_speed, 6, 2, left_speed);
  dtostrf(goal_right_speed, 6, 2, right_speed);

  sprintf(log_msg, "Left speed.->%s - Right speed.->%s", left_speed, right_speed);
  nh.logwarn(log_msg);
}

void setPIDValues() {
  bool params_error = false;
  float left_pid[3], right_pid[3];
  if(!nh.getParam("/left_pid", left_pid, 3))   { params_error = true; }
  if(!nh.getParam("/right_pid", right_pid, 3)) { params_error = true; }
  if(params_error) { nh.logerror("File: ~/RemoteMinibot/catkin_ws/src/hardware/params/pid.yaml did not load values, check it please"); }

  motors.setLeftPID(left_pid);
  motors.setRightPID(right_pid);
}

void publish_encoders(){
  long encoder_data[2];
  encoders_msg.data_length = 2;
  encoder_data[0] = encoders.get_left_count();
  encoder_data[1] = encoders.get_right_count();
  encoders_msg.data = encoder_data;
  
  encodersPub.publish(&encoders_msg);
}

void publish_sensors_data(){
  sensors.read();

  line_sensors_msg.data_length  = 3;
  light_sensors_msg.data_length = 8;
  sharp_sensors_msg.data_length = 7;
  
  batt_perc_msg.data = sensors.get_battery_status();
  line_sensors_msg.data  = sensors.get_line_status();
  light_sensors_msg.data = sensors.get_lights_status();
  sharp_sensors_msg.data = sensors.get_sharps_status();

  battPercPub.publish(&batt_perc_msg);
  lineSensorsPub.publish(&line_sensors_msg);
  lightSensorsPub.publish(&light_sensors_msg);
  sharpSensorsPub.publish(&sharp_sensors_msg);  
}

void motorsSpeedCallback(const std_msgs::Float32MultiArray& msg){
  printPIDVars(msg.data[0], msg.data[1]);
  motors.setSpeeds(msg.data[0], msg.data[1]);
}

void setup() {
  nh.getHardware()->setBaud(BAUD);
  nh.initNode();

  setPIDValues();
  nh.advertise(battPercPub);
  nh.advertise(encodersPub);
  nh.advertise(lineSensorsPub);
  nh.advertise(lightSensorsPub);
  nh.advertise(sharpSensorsPub);
  nh.subscribe(subMotorsSpeed);
}

void loop() {
  motors.setEncodersCount(encoders.get_left_count(), encoders.get_right_count());
  publish_encoders();
  publish_sensors_data();

  nh.spinOnce();
  delay(20);
}
