// Cabeceras para ros
#include <ros/ros.h>
// Cabeceras para mensajes estandar
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
// Cabeceras para el tipo de mensaje twist
#include <geometry_msgs/Twist.h>
// Cabeceras para el estado de los joints
#include <sensor_msgs/JointState.h>
// Cabeceras para realizar las transformaciones
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
// Cabeceras para la Odometria
#include <nav_msgs/Odometry.h>

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
	//std::cout << "Reciving command robot velociy" << std::endl;
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
	//Inicializacion del nodo
	ros::init(argc, argv, "mobile_base_node");
	//Se crea el mando del nodo
	ros::NodeHandle nh;
	//Se crea el objeto que nos permite realizar el muestreo de los mensajes
	ros::Rate rate(50);

	// Se crean los subscriptores de los mensajes
	ros::Subscriber subEncoders = nh.subscribe("/encoders_data", 1, callbackEncoders);
	ros::Subscriber subCmdVel = nh.subscribe("/cmd_vel", 1, callbackCmdVel);

	// Publicador del estado de los joints
	ros::Publisher pubJointState = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
	// Publicador de la posicion del robot con base en odometria
	ros::Publisher pubOdom  = nh.advertise<nav_msgs::Odometry>("odom", 50);
	// Publicador del las velocidades de cada motor	
	pubSpeeds = nh.advertise<std_msgs::Float32MultiArray>("/speed_motors", 1);
	
	// Nombre de los joints
	std::string jointNames[2] = {"left_wheel_joint_connect", "right_wheel_joint_connect"};
	float jointPositions[2] = {0.0, 0.0};
	sensor_msgs::JointState jointState;

	// Datos de odometria
	nav_msgs::Odometry odom;

	// Se asignan los nombres de los joints y las dimensiones 
	jointState.name.insert(jointState.name.begin(), jointNames, jointNames + 2);
	jointState.position.insert(jointState.position.begin(), jointPositions, jointPositions + 2);

	// El objeto que envía las transformaciones
	tf::TransformBroadcaster br;

	// Loop de ros
	while(ros::ok()){
		computeOdom();

		// Transformaciones
		tf::Transform transform;
		transform.setOrigin(tf::Vector3(robotX, robotY, 0));
		tf::Quaternion q;
		q.setRPY(0, 0, robotT);
		transform.setRotation(q);

		// Se envia la transformaciones del base_link al odom
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
		
		// Odometria 
		odom.header.stamp = ros::Time::now();
		odom.header.frame_id = "odom";
		geometry_msgs::Quaternion odom_quad = tf::createQuaternionMsgFromYaw(robotT);

		odom.pose.pose.position.x = robotX;
		odom.pose.pose.position.y = robotY;
		odom.pose.pose.position.z =    0.0;
		odom.pose.pose.orientation = odom_quad;


		// Se Publican el estado de los joints
		pubJointState.publish(jointState);
		// Se Publica la odometria
		pubOdom.publish(odom);
		rate.sleep();
		ros::spinOnce();
	}
}
