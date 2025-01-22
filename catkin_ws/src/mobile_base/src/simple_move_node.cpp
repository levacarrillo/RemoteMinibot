#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <mobile_base/MoveMinibot.h>
#include <mobile_base/OdomSetPoint.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <mobile_base/mobile_base_utils.h>

struct robotPose {
  float x;
  float y;
  float th;
  float magnitude;
} curr, goal, error;

enum State {
    SM_MOVE_ROBOT,
    SM_CORRECT_ANGLE,
    SM_FINISH_MOVEMENT
};

ros::Publisher pubCmdVel;
ros::ServiceClient client;

bool setInitialPosition() {
    mobile_base::OdomSetPoint srv;
    srv.request.robot_x = curr.x;
    srv.request.robot_y = curr.y;
    srv.request.robot_w = curr.th;

    curr.x  = 0.0;
    curr.y  = 0.0;
    curr.th = 0.0;
    if (!client.call(srv)) {
        ROS_ERROR("simple_move.->FAILED TO GET SERVICE /mobile_base/odom_set_point");
        return false;
    }

    return true;
}

robotPose getGoalPose(mobile_base::MoveMinibot::Request req) {
    std::cout << "simple_move.-> REQ. theta: " << req.theta << "\tdistance: " << req.distance << std::endl;
    robotPose goalPose;
    goalPose.x  = req.distance * cos(req.theta);
    goalPose.y  = req.distance * sin(req.theta);
    goalPose.th = req.theta;
    goalPose.magnitude = req.distance;

    if (goalPose.th >  M_PI)  goalPose.th -= 2 * M_PI;
    if (goalPose.th <= -M_PI) goalPose.th += 2 * M_PI;

    return goalPose;
}

robotPose getErrorPose() {
    robotPose errorPose;
    errorPose.x = fabs(goal.x) - fabs(curr.x);
    errorPose.y = fabs(goal.y) - fabs(curr.y);
    errorPose.th = goal.th - curr.th;
    errorPose.magnitude = sqrt(pow(errorPose.x, 2) + pow(errorPose.y, 2));
    if (goal.magnitude < 0) errorPose.magnitude = - errorPose.magnitude;

    return errorPose;
}

float roundNumber(double num) {
    return std::round(num * 1000) / 1000;
}

bool movementCallback(mobile_base::MoveMinibot::Request &req, mobile_base::MoveMinibot::Response &res) {
    while(!setInitialPosition()) {}
    goal = getGoalPose(req);
    ros::Rate rate(50);

    State state = SM_CORRECT_ANGLE;

    float last_error = 0;
    int current_time = 0;
    bool goal_exceeded = false;
    res.done = req.distance == 0.0 && req.theta == 0.0;

    while(ros::ok() && !res.done) {
        std::cout << "-------------------------------------------------------------------" << std::endl;
        // std::cout << "simple_move.-> ANGLE_TOLERANCY: " << ANGLE_TOLERANCY << "\tDISTANCE_TOLERANCY: " << DISTANCE_TOLERANCY << std::endl;
        std::cout << "simple_move.-> STATE MACHINE: " << state << std::endl;
        // std::cout << std::endl;
        // std::cout << "simple_move.-> Goal->     x:" << roundNumber(goal.x)  << "\ty:" << roundNumber(goal.y)  << "\tth:" << roundNumber(goal.th * 180 / M_PI) << "\tmag:" << roundNumber(goal.magnitude) << std::endl;
        std::cout << "simple_move.-> Currrent-> x:" << roundNumber(curr.x)  << "\ty:" << roundNumber(curr.y)  << "\tth:" << roundNumber(curr.th * 180 / M_PI) << "\tmag:" << roundNumber(curr.magnitude) << std::endl;
        std::cout << "simple_move.-> Error->    x:" << roundNumber(error.x) << "\ty:" << roundNumber(error.y) << "\tth:" << roundNumber(error.th * 180 / M_PI) << "\tmag:" << roundNumber(error.magnitude) << std::endl;
        // std::cout << "simple_move.-> Time execution->" << current_time << "ms" << std::endl;
        // std::cout << std::endl;

        if (!isRunning()) {
            res.done = true;
            std::cout << "************* simple_move.-> MOVEMENT STOPPED **************" << std::endl;
            break;
        }

        error = getErrorPose();

        switch(state) {
            case SM_CORRECT_ANGLE:
                std::cout << "simple_move.-> current_angle_error: " << roundNumber(error.th * 180 / M_PI) << "\tlast_error.->" << roundNumber(last_error * 180 / M_PI) << std::endl;
                if (last_error != 0 && error.th*last_error < 0) {
                    goal_exceeded = true;
                    std::cout << "simple_move.-> STATE MACHINE " << state << "\tHAS EXCEEDED GOAL" << std::endl;
                }
                if(!goal_exceeded && abs(error.th) >= ANGLE_TOLERANCY) {
                    pubCmdVel.publish(getAngularVelocity(goal.th, error.th));
                    last_error = error.th;
                }
                else {
                    last_error = 0; // RESTART ERROR COUNT
                    goal_exceeded = false;
                    if (req.distance == 0.0) {
                        state = SM_FINISH_MOVEMENT;
                    } else {
                        state = SM_MOVE_ROBOT;
                    }
                    pubCmdVel.publish(stop());
                }
            break;
            case SM_MOVE_ROBOT:
                std::cout << "simple_move.-> current_linear_error: " << roundNumber(error.magnitude) << "\tlast_error.->" << last_error << std::endl;

                if (last_error != 0) {
                    if (last_error > 0) {
                        if (last_error < error.magnitude) {
                            goal_exceeded = true;
                            std::cout << "simple_move.-> STATE MACHINE " << state << "\tHAS EXCEEDED GOAL" << std::endl;
                        }
                    } else {
                        if (last_error > error.magnitude) {
                            goal_exceeded = true;
                            std::cout << "simple_move.-> STATE MACHINE " << state << "\tHAS EXCEEDED GOAL" << std::endl;
                        }
                    }
                }
                if(!goal_exceeded && abs(error.magnitude) >= DISTANCE_TOLERANCY) {
                    pubCmdVel.publish(getLinearVelocity(curr.magnitude, goal.magnitude, error.magnitude));
                    last_error = error.magnitude;
                }
                else {
                    last_error = 0;
                    goal_exceeded = false;
                    state = SM_FINISH_MOVEMENT;
                    pubCmdVel.publish(stop());
                }
            break;
            case SM_FINISH_MOVEMENT:
                pubCmdVel.publish(stop());
                std::cout << "simple_move.-> Success distance reached! with dist.->" << req.distance << "  and angle.->" << req.theta << std::endl;
                res.done = true;
            break;

            default:
                std::cout << "simple_move.-> An unexpected error has occurred :(" << std::endl;
        }

        current_time += 1;
        if (current_time > MAX_TIME_LIMIT) {
            current_time = 0;
            state = SM_FINISH_MOVEMENT;
            std::cout << "simple_move.-> STATE MACHINE HAS EXCEEDED THE MAX TIME" << std::endl;
        }

        ros::spinOnce();
	    rate.sleep();
    }

    pubCmdVel.publish(stop());
    return res.done;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // ROS_INFO("Seq: [%d]", msg->header.seq);
    // ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
    // ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    // ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    curr.x  = msg->pose.pose.position.x;
    curr.y  = msg->pose.pose.position.y;
    curr.th = yaw;
    curr.magnitude = sqrt(pow(curr.x, 2) + pow(curr.y, 2));
}

int main(int argc, char **argv) {
    std::cout << "Starting simple_move_node by Luis Nava..." << std::endl;
    ros::init(argc, argv, "simple_move_node");
    ros::NodeHandle nh;
    if(!setParameters()) return -1;

    pubCmdVel = nh.advertise<geometry_msgs::Twist>("/mobile_base/cmd_vel", 1);
    client = nh.serviceClient<mobile_base::OdomSetPoint>("/mobile_base/odom_set_point");
    ros::ServiceServer movementService = nh.advertiseService("/mobile_base/move_to_pose", movementCallback);
    ros::Subscriber odomSub = nh.subscribe("/mobile_base/odom", 1000, odomCallback);

    ros::spin();
    return 0;
}
