#include <ros.h>
#include <Sensors.h>
#include <Encoders.h>
#include <Motors.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

#define BAUD 200000

ros::NodeHandle nh;

Motors motors;
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
   
  float left_speed  = msg.data[0];
  float right_speed = msg.data[1];
  // timer = 0;
  nh.logwarn("left_speed->");
  nh.logwarn("right_speed->");
  motors.move(1, 255, 1, 255);
}

void setup() {
  nh.getHardware()->setBaud(BAUD);
  nh.initNode();

  nh.advertise(battPercPub);
  nh.advertise(encodersPub);
  nh.advertise(lineSensorsPub);
  nh.advertise(lightSensorsPub);
  nh.advertise(sharpSensorsPub);
  nh.subscribe(subMotorsSpeed);
}

void loop() {
    publish_encoders();
    publish_sensors_data();
    // nh.logdebug("Debug statement");
    // nh.loginfo("Program info");
    // nh.logwarn("Warnings");
    // nh.logerror("Errors...");
    // nh.logfatal("Fatalities");

    nh.spinOnce();
    delay(20);
}
