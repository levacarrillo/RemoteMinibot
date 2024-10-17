#include <ros/ros.h>
#include <mobile_base/MoveMinibot.h>

bool moveCallback(mobile_base::MoveMinibot::Request &req, mobile_base::MoveMinibot::Response &res) {
    std::cout << "Distance received->"<< req.distance << std::endl;
    std::cout << "Angle received->" << req.theta << std::endl;

    res.done = true;
    return res.done;
}

int main(int argc, char ** argv){
    std::cout << "Starting mobile_base_node by Luis Nava..." << std::endl;
	ros::init(argc, argv, "mobile_base_node");
	ros::NodeHandle nh;
	ros::Rate rate(50);

    ros::ServiceServer move = nh.advertiseService("move_minibot", moveCallback);

	while(ros::ok()){

		rate.sleep();
		ros::spinOnce();
	}
}
