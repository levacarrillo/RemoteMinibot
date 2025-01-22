#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
// #include <sensor_msgs/JointState.h>
// #include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TransformStamped.h>
// #include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mobile_base/mobile_base_utils.h>
#include <mobile_base/OdomSetPoint.h>


#define BASE_WIDTH 0.095
#define TICKS_PER_METER 4442.0
#define ANGLE_CORRECTION 1.0

long currEncoderLeft  = 0;
long currEncoderRight = 0;
long lastEncoderLeft  = 0;
long lastEncoderRight = 0;
double robotX = 0.0, robotY = 0.0, robotT = 0.0;
double vx = 0.0, vy = 0.0, vth = 0.0;

ros::Publisher pubOdom;
ros::Publisher pubSpeeds;

void callbackCmdVel(const geometry_msgs::Twist::ConstPtr &msg) {
    // std::cout << "Received command robot velociy" << std::endl;
    // std::cout << "mobile_base.->linear x: " << msg->linear.x << "\tangular z: "<< msg->angular.z << std::endl;
    float rightSpeed = msg->linear.x + msg->angular.z * BASE_WIDTH / 2.0;
    float leftSpeed  = msg->linear.x - msg->angular.z * BASE_WIDTH / 2.0;
	
    std_msgs::Float32MultiArray speeds_msg;
    speeds_msg.data.resize(2);
    speeds_msg.data[0] =  leftSpeed;
    speeds_msg.data[1] = rightSpeed;
    pubSpeeds.publish(speeds_msg);
}

void callbackSensors(const std_msgs::Float32MultiArray::ConstPtr &msg){
    currEncoderLeft  = msg->data[15];
    currEncoderRight = msg->data[16];

    vx  = (msg->data[9] + msg->data[11]) / 2;
    vy  = 0.0;
    vth = (msg->data[9] - msg->data[11]) / 2;
}

void publishOdom() {
    // BROADCAST FRAMES
    static tf2_ros::TransformBroadcaster br;
    br.sendTransform(getTFStamped("odom", "base_link", robotX, robotY, robotT));
    // PUBLISH ODOM DATA
    tf2::Quaternion q;
    q.setRPY(0, 0, robotT);
    geometry_msgs::Quaternion odom_quad = tf2::toMsg(q);

    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = robotX;
    odom.pose.pose.position.y = robotY;
    odom.pose.pose.position.z =    0.0;
    odom.pose.pose.orientation = odom_quad;

    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x  = vx;
    odom.twist.twist.linear.y  = vy;
    odom.twist.twist.angular.z = vth;

    pubOdom.publish(odom);
}

bool odomCallback(mobile_base::OdomSetPoint::Request &req, mobile_base::OdomSetPoint::Response &res) {
    // std::cout << "mobile_base.-> req.x: " << req.robot_x <<  "\treq.y: " << req.robot_y << "\treq.w: " << req.robot_w << std::endl;
    static tf2_ros::StaticTransformBroadcaster br;
    br.sendTransform(getTFStamped("map", "odom",  req.robot_x, req.robot_y, req.robot_w));

    robotX = 0.0;
    robotY = 0.0;
    robotT = 0.0;

    res.done = true;
    return res.done;
}

float normalizeAngle(float angle){
    while(angle > M_PI)
        angle -= 2 * M_PI;
    while(angle < -M_PI)
        angle += 2 * M_PI;
    return angle;
}

void computeOdom(){
	long leftTicks = currEncoderLeft - lastEncoderLeft;
	long rightTicks = currEncoderRight - lastEncoderRight;
	lastEncoderLeft = currEncoderLeft;
	lastEncoderRight = currEncoderRight;

	double distLeft = leftTicks / TICKS_PER_METER;
	double distRight = rightTicks / TICKS_PER_METER;

	double deltaTheta = ANGLE_CORRECTION * (distRight - distLeft)/BASE_WIDTH;
	double distX = (distLeft + distRight) / 2.0;

	robotT = normalizeAngle(robotT + deltaTheta);
	robotX += distX * cos(robotT);
	robotY += distX * sin(robotT);
    double magnitude = sqrt(pow(robotX, 2) + pow(robotY, 2));
    // std::cout << "mobile_base.-> robotX: " << robotX << "\trobotY: " << robotY << "\tmagnitude: " << magnitude << "\trobotT: " << robotT * 180 / M_PI << std::endl;
    // std::cout << "mobile_base.-> robotX: " << robotX <<  "\trobotY: " << robotY << "\trobotT: " << robotT * 180 / M_PI << std::endl;
}

int main(int argc, char ** argv) {
    std::cout << "Starting mobile_base_node by Luis Nava..." << std::endl;
	ros::init(argc, argv, "mobile_base_node");
	ros::NodeHandle nh;
    ros::Rate rate(2);

    ros::Subscriber subCmdVel = nh.subscribe("/mobile_base/cmd_vel", 1, callbackCmdVel);
    ros::Subscriber subEncoders = nh.subscribe("/hardware/sensors", 1, callbackSensors);
    // ros::Publisher pubJointState   = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
    ros::ServiceServer odomService = nh.advertiseService("/mobile_base/odom_set_point", odomCallback);
    pubSpeeds = nh.advertise<std_msgs::Float32MultiArray>("/hardware/speed_motors", 1);
    pubOdom  = nh.advertise<nav_msgs::Odometry>("odom", 1);

    if (!setInitialPose()) { return -1; }

	// std::string jointNames[2] = {"left_wheel_joint_connect", "right_wheel_joint_connect"};
	// float jointPositions[2] = {0.0, 0.0};

	// sensor_msgs::JointState jointState;

	// jointState.name.insert(jointState.name.begin(), jointNames, jointNames + 2);
	// jointState.position.insert(jointState.position.begin(), jointPositions, jointPositions + 2);
    
    static tf2_ros::StaticTransformBroadcaster br;
    br.sendTransform(getTFStamped("map", "odom", robot_pose_x, robot_pose_y, robot_pose_w));
    
    while(ros::ok()) {
        computeOdom();
        // pubJointState.publish(jointState);
        publishOdom();
        
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
