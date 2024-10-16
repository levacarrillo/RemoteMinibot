#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// #include <tf/transform_broadcaster.h>
// #include <tf/transform_listener.h>

#define TICKS_PER_METER 4442.0

#define BASE_WIDTH 0.095
#define ANGLE_CORRECTION 1.0

long currEncoderLeft = 0;
long currEncoderRight = 0;
long lastEncoderLeft = 0;
long lastEncoderRight = 0;
double robotX = 0.0, robotY = 0.0, robotT = 0.0;

ros::Publisher pubSpeeds;

void callbackEncoders(const std_msgs::Int32MultiArray::ConstPtr &msg){
	currEncoderLeft  = msg->data[0];
	currEncoderRight = msg->data[1];
}

void callbackCmdVel(const geometry_msgs::Twist::ConstPtr & msg){
	std::cout << "Recived command robot velociy" << std::endl;
	float rightSpeed = msg->linear.x + msg->angular.z * BASE_WIDTH / 2.0;
	float leftSpeed = msg->linear.x - msg->angular.z * BASE_WIDTH / 2.0;

	std_msgs::Float32MultiArray msgSend;
	msgSend.data.resize(2);
	msgSend.data[0] =  leftSpeed;
	msgSend.data[1] = rightSpeed;
	pubSpeeds.publish(msgSend);
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
}

int main(int argc, char ** argv){
	std::cout << "Starting mobile_base_node by Luis Nava..." << std::endl;
	ros::init(argc, argv, "mobile_base_node");
	ros::NodeHandle nh;
	ros::Rate rate(50);

	ros::Subscriber subEncoders = nh.subscribe("/encoders_data", 1, callbackEncoders);
	ros::Subscriber subCmdVel = nh.subscribe("/cmd_vel", 1, callbackCmdVel);

	ros::Publisher pubJointState = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
	ros::Publisher pubOdom  = nh.advertise<nav_msgs::Odometry>("odom", 50);

	pubSpeeds = nh.advertise<std_msgs::Float32MultiArray>("/speed_motors", 1);
	
	std::string jointNames[2] = {"left_wheel_joint_connect", "right_wheel_joint_connect"};
	float jointPositions[2] = {0.0, 0.0};
	sensor_msgs::JointState jointState;

	nav_msgs::Odometry odom;

	jointState.name.insert(jointState.name.begin(), jointNames, jointNames + 2);
	jointState.position.insert(jointState.position.begin(), jointPositions, jointPositions + 2);

	// tf::TransformBroadcaster br;
	tf2_ros::TransformBroadcaster br;

	while(ros::ok()){
		computeOdom();

		geometry_msgs::TransformStamped transformStamped;

		transformStamped.header.stamp = ros::Time::now();
		transformStamped.header.frame_id = "odom";
		transformStamped.child_frame_id = "base_link";
		
		transformStamped.transform.translation.x = robotX;
		transformStamped.transform.translation.y = robotY;
		transformStamped.transform.translation.z = 0.0;

		tf2::Quaternion q;
		q.setRPY(0, 0, robotT);

		transformStamped.transform.rotation.x = q.x();
		transformStamped.transform.rotation.y = q.y();
		transformStamped.transform.rotation.z = q.z();
		transformStamped.transform.rotation.w = q.w();
		
		br.sendTransform(transformStamped);
		

		// tf::Transform transform;
		// transform.setOrigin(tf::Vector3(robotX, robotY, 0));
		// tf::Quaternion q;
		// q.setRPY(0, 0, robotT);
		// transform.setRotation(q);

		// br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
		
		// geometry_msgs::Quaternion odom_quad = tf::createQuaternionMsgFromYaw(robotT);
		geometry_msgs::Quaternion odom_quad = tf2::toMsg(q);

		odom.header.stamp = ros::Time::now();
		odom.header.frame_id = "odom";
		odom.pose.pose.position.x = robotX;
		odom.pose.pose.position.y = robotY;
		odom.pose.pose.position.z =    0.0;
		odom.pose.pose.orientation = odom_quad;

		pubJointState.publish(jointState);
		pubOdom.publish(odom);

		rate.sleep();
		ros::spinOnce();
	}
}
