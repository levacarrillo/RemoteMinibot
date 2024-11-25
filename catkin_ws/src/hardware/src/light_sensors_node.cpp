#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <hardware/LightReadings.h>

float light_readings[8];

bool light_callback(hardware::LightReadings::Request &req, hardware::LightReadings::Response &res) {
    int sensor_max_intensity = 0;
    float max_intensity = 0.0f;

    for(size_t i=0; i<8; i++) {
        res.light_readings[i] = light_readings[i];
        if (light_readings[i] > max_intensity) {
            max_intensity = light_readings[i];
            sensor_max_intensity = i;
        }
    }
    res.sensor_max_intensity = sensor_max_intensity;
    res.max_intensity = max_intensity;
    return true;
}

void subs_callback(const std_msgs::Int16MultiArray::ConstPtr& msg) {
    for(size_t i=0; i<8; i++) {
        light_readings[i] = msg->data[i];
        // std::cout << "light_readings: " << i << " -> " << light_readings[i] << std::endl;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "light_sensors_node");
    ros::NodeHandle nh;    

    ros::ServiceServer service = nh.advertiseService("/hardware/light_readings", light_callback);
    ros::Subscriber subs = nh.subscribe<std_msgs::Int16MultiArray>("/hardware/light_sensors", 1, subs_callback);
    ros::spin();
    return 0;
}