#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <mobile_base/MoveMinibot.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mobile_base/profiles.h>;

struct robotPose {
  float x;
  float y;
  float th;
  float magnitude;
} init, goal, curr, error;

enum State {
    SM_MOVE_ROBOT,
    SM_CORRECT_ANGLE,
    SM_FINISH_MOVEMENT
};

float ANGLE_TOLERANCY    = 0.10;
float DISTANCE_TOLERANCY = 0.08;

ros::Publisher pubCmdVel;

float normalizeAngle(float angle) {
    if (angle < 0) angle += 2 * M_PI;
    return angle;
}

robotPose getAbsolutePose(tf2_ros::Buffer& tfBuffer) {
    robotPose absolutePose;
    try{
        geometry_msgs::TransformStamped transformStamped;
        transformStamped = tfBuffer.lookupTransform("odom", "base_link", ros::Time::now(), ros::Duration(0.1));
        tf2::Quaternion q;
        tf2::fromMsg(transformStamped.transform.rotation, q);
        double roll, pitch, yaw;
        tf2::getEulerYPR(q, yaw, pitch, roll);
        
        absolutePose.x = transformStamped.transform.translation.x;
        absolutePose.y = transformStamped.transform.translation.y;
        absolutePose.th = yaw;
        absolutePose.magnitude = sqrt(pow(absolutePose.x, 2) + pow(absolutePose.y, 2));

    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    return absolutePose;
}

robotPose getGoalPose(mobile_base::MoveMinibot::Request req) {
    robotPose goalPose;
    goalPose.x  = req.distance * cos(req.theta);
    goalPose.y  = req.distance * sin(req.theta);
    goalPose.th = req.theta;
    goalPose.magnitude = req.distance;

    if (goalPose.th >  M_PI)  goalPose.th -= 2 * M_PI;
    if (goalPose.th <= -M_PI) goalPose.th += 2 * M_PI;
    return goalPose;
}

robotPose getCurrentPose(robotPose absPose) {
    robotPose currentPose;
    currentPose.x  = absPose.x  - init.x;
    currentPose.y  = absPose.y  - init.y;
    if (init.th + goal.th < -M_PI || init.th + goal.th > M_PI ) {
        currentPose.th = normalizeAngle(absPose.th) - normalizeAngle(init.th);
    } else {
        currentPose.th = absPose.th - init.th;
    }
    currentPose.magnitude = sqrt(pow(currentPose.x, 2) + pow(currentPose.y, 2));
    return currentPose;
}

robotPose getErrorPose() {
    robotPose errorPose;
    errorPose.x = goal.x - curr.x;
    errorPose.y = goal.y - curr.y;
    errorPose.th = goal.th - curr.th;
    errorPose.magnitude = sqrt(pow(errorPose.x, 2) + pow(errorPose.y, 2));
    if (goal.magnitude < 0) errorPose.magnitude = - errorPose.magnitude;
    return errorPose;
}

bool moveCallback(mobile_base::MoveMinibot::Request &req, mobile_base::MoveMinibot::Response &res) {
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate rate(10);
    
    curr = getAbsolutePose(tfBuffer);
    init = getAbsolutePose(tfBuffer);
    goal = getGoalPose(req);

    State state = SM_CORRECT_ANGLE;

    while(ros::ok() && !res.done) {
        curr  = getCurrentPose(getAbsolutePose(tfBuffer));
        error = getErrorPose();
        std::cout << "-------------------------------------------------------------------" << std::endl;
        std::cout << "initial:  x->" << init.x  << "\ty->" << init.y  << "\tth->" << init.th << "\tmag->" << init.magnitude << std::endl;
        std::cout << "currrent: x->" << curr.x  << "\ty->" << curr.y  << "\tth->" << curr.th << "\tmag->" << curr.magnitude << std::endl;
        std::cout << "goal:     x->" << goal.x  << "\ty->" << goal.y  << "\tth->" << goal.th << "\tmag->" << goal.magnitude << std::endl;
        std::cout << "error:    x->" << error.x << "\ty->" << error.y << "\tth->" << error.th << "\tmag->" << error.magnitude << std::endl;
        std::cout << std::endl;

        switch(state) {
            case SM_CORRECT_ANGLE:
                if(abs(error.th) > ANGLE_TOLERANCY) {
                    pubCmdVel.publish(getAngularVelocity(error.th));
                } else {
                    state = SM_MOVE_ROBOT;
                }
            break;
            case SM_MOVE_ROBOT:
                if(abs(error.magnitude) > DISTANCE_TOLERANCY) {
                    pubCmdVel.publish(getLinearVelocity(curr.magnitude, goal.magnitude, error.th));
                } else {
                    current_speed = 0;
                    state = SM_FINISH_MOVEMENT;
                }
            break;
            case SM_FINISH_MOVEMENT:
                pubCmdVel.publish(stop());
                std::cout << "Success distance reached! with dist.->" << req.distance << "  and angle.->" << req.theta << std::endl;
                res.done = true;
            break;
            default:
                std::cout << "An unexpected error has occurred :(" << std::endl;
        }

	    rate.sleep();
    }
    pubCmdVel.publish(stop());
    return res.done;
}

int main(int argc, char **argv) {
    std::cout << "Starting move_minibot_node by Luis Nava..." << std::endl;
    ros::init(argc, argv, "move_minibot_node");
    ros::NodeHandle nh;
    if(!setParameters()) return -1;

    pubCmdVel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::ServiceServer moveService = nh.advertiseService("move_robot", moveCallback);

    ros::spin();
    return 0;
}
