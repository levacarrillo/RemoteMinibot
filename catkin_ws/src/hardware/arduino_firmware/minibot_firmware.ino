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
std_msgs::Float32MultiArray curr_speeds_msg;
std_msgs::Int16MultiArray line_sensors_msg;
std_msgs::Int16MultiArray light_sensors_msg;
std_msgs::Int16MultiArray sharp_sensors_msg;

void motorsSpeedCallback(const std_msgs::Float32MultiArray& msg);

ros::Publisher battPercPub("/battery_data", &batt_perc_msg);
ros::Publisher encodersPub("/encoders_data", &encoders_msg);
ros::Publisher currSpeedsPub("/curr_speeds", &curr_speeds_msg);
ros::Publisher lineSensorsPub("/line_sensors", &line_sensors_msg);
ros::Publisher lightSensorsPub("/light_sensors", &light_sensors_msg);
ros::Publisher sharpSensorsPub("/sharp_sensors", &sharp_sensors_msg);
ros::Subscriber<std_msgs::Float32MultiArray> subMotorsSpeed("/speed_motors", motorsSpeedCallback);

bool getPIDValues() {
  bool params_error = false;
  float left_pid[3] = {0, 0, 0}, right_pid[3] = {0, 0, 0};
  if(!nh.getParam("/hardware/left_pid", left_pid, 3))   { params_error = true; }
  if(!nh.getParam("/hardware/right_pid", right_pid, 3)) { params_error = true; }
  if(params_error) {
    return false;
  }
  motors.setLeftPID(left_pid);
  motors.setRightPID(right_pid);
  return true;
}

void publish_encoders(){
  long encoder_data[2];
  encoders_msg.data_length = 2;
  encoder_data[0] = encoders.get_left_count();
  encoder_data[1] = encoders.get_right_count();
  encoders_msg.data = encoder_data;
  
  encodersPub.publish(&encoders_msg);
}

void publish_curr_speeds(){
  curr_speeds_msg.data_length = 8;
  float* speeds = motors.getCurrVelocities();
  float speeds_data[8];
  for (int i=0; i<8; i++) {
    speeds_data[i] = speeds[i];
  }
  curr_speeds_msg.data = speeds_data;
  currSpeedsPub.publish(&curr_speeds_msg);
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
  motors.setSpeeds(msg.data[0], msg.data[1]);
}

void setup() {
  nh.getHardware()->setBaud(BAUD);
  nh.initNode();

  while(!getPIDValues()) {
    nh.logwarn("Searching PID values from: ~/RemoteMinibot/catkin_ws/src/hardware/params/pid.yaml");
    delay(10);
  }

  nh.advertise(battPercPub);
  nh.advertise(encodersPub);
  nh.advertise(currSpeedsPub);
  nh.advertise(lineSensorsPub);
  nh.advertise(lightSensorsPub);
  nh.advertise(sharpSensorsPub);
  nh.subscribe(subMotorsSpeed);
}

void loop() {
  getPIDValues();
  motors.setEncodersCount(encoders.get_left_count(), encoders.get_right_count());
  publish_encoders();
  publish_curr_speeds();
  publish_sensors_data();

  nh.spinOnce();
  delay(10);
}
