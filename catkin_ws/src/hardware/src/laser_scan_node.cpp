#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <sensor_msgs/LaserScan.h>

#define first_sensor_id 2
#define last_sensor_id 5

// CONSTANTS TO MODEL DATA
#define a 9.609
#define b -0.832
#define c 0.08

float sharp_distance[3];

void subs_callback(const std_msgs::Int16MultiArray::ConstPtr& msg) {
    for(size_t i=first_sensor_id; i<last_sensor_id; i++) {
        int sharp_reading = msg->data[i];
        sharp_distance[i-first_sensor_id] = a * pow(sharp_reading, b) + c;
        // std::cout << "sharp_distance: " << i << " -> " << sharp_distance[i-first_sensor_id] << std::endl;
        if(sharp_distance[i - first_sensor_id] > 0.7) {
            sharp_distance[i - first_sensor_id] = 0.7;
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "laser_scan_node");
    ros::NodeHandle nh;
    ros::Rate loop(50);

    ros::Publisher pub = nh.advertise<sensor_msgs::LaserScan>("/scan", 50);
    ros::Subscriber subs = nh.subscribe<std_msgs::Int16MultiArray>("/sharp_sensors", 1, subs_callback);
    
    unsigned int num_readings = 3;
    double laser_frequency = 40;
    double ranges[num_readings];
    double intensities[num_readings];

    while(ros::ok()) {

        for(size_t i=0; i<num_readings; i++) {
            ranges[i] = sharp_distance[i];
            intensities[i] = 100;
        }

        ros::Time scan_time = ros::Time::now();
        sensor_msgs::LaserScan scan;
        scan.header.stamp = scan_time;
        scan.header.frame_id = "laser_link";
        scan.angle_min = - M_PI / 4;
        scan.angle_max = M_PI;

        scan.angle_increment = M_PI / 4;
        scan.time_increment = 1 / (laser_frequency * num_readings);
        scan.range_min = 0.0;
        scan.range_max = 100.0;

        scan.ranges.resize(num_readings);
        scan.intensities.resize(num_readings);

        for(unsigned int i=0; i< num_readings; ++i) {
            scan.ranges[i] = ranges[i];
            scan.intensities[i] = intensities[i];
        }

        pub.publish(scan);
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}