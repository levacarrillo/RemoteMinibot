#include <ros.h>
#include <std_msgs/String.h>

#define BAUD 76800

ros::NodeHandle nh;

std_msgs::String str_msg;

ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "Hello ROS";

void setup() {
    nh.getHardware()->setBaud(BAUD);
    nh.initNode();
    nh.advertise(chatter);
}

void loop() {
    // nh.logdebug("Debug statement");
    // nh.loginfo("Program info");
    // nh.logwarn("Warnings");
    // nh.logerror("Errors...");
    // nh.logfatal("Fatalities");

    str_msg.data = hello;
    chatter.publish(&str_msg);
    nh.spinOnce();
    delay(1);
}
