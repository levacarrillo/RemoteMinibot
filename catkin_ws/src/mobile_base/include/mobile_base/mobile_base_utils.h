#ifndef MOBILE_BASE_UTILS_H
#define MOBILE_BASE_UTILS_H

#include <iostream>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/static_transform_broadcaster.h>

float current_speed;

float max_linear_vel;
float speed_increment;
float linear_alpha;

float MAX_TIME_LIMIT;
float ANGLE_TOLERANCY;
float DISTANCE_TOLERANCY;

float max_angular_vel;
float angular_alpha;

float robot_pose_x;
float robot_pose_y;
float robot_pose_w;

// SETTING UP PARAMETERS
bool setInitialPose() {
    if(ros::param::has("/mobile_base/robot_pose_x")) ros::param::get("/mobile_base/robot_pose_x", robot_pose_x);
    else { ROS_ERROR("There's no parameter for /mobile_base/robot_pose_x"); return false; }

    if(ros::param::has("/mobile_base/robot_pose_y")) ros::param::get("/mobile_base/robot_pose_y", robot_pose_y);
    else { ROS_ERROR("There's no parameter for /mobile_base/robot_pose_y"); return false; }

    if(ros::param::has("/mobile_base/robot_pose_w")) ros::param::get("/mobile_base/robot_pose_w", robot_pose_w);
    else { ROS_ERROR("There's no parameter for /mobile_base/robot_pose_w"); return false; }

    return true;
}

bool setParameters() {
    // PARAMETERS FOR LIMIT TIME
    if(ros::param::has("/mobile_base/max_time_limit")) ros::param::get("/mobile_base/max_time_limit", MAX_TIME_LIMIT);
    else { ROS_ERROR("profiles.->THERE'S NO PARAMETERS FOR /mobile_base/max_time_limit"); return false; }
    // PARAMETERS FOR POSITION'S TOLERANCY
    if(ros::param::has("/mobile_base/angle_tolerancy")) ros::param::get("/mobile_base/angle_tolerancy", ANGLE_TOLERANCY);
    else { ROS_ERROR("profiles.->THERE'S NO PARAMETERS FOR /mobile_base/angle_tolerancy"); return false; }

    if(ros::param::has("/mobile_base/distance_tolerancy")) ros::param::get("/mobile_base/distance_tolerancy", DISTANCE_TOLERANCY);
    else { ROS_ERROR("profiles.->THERE'S NO PARAMETERS FOR /mobile_base/distance_tolerancy"); return false; }

    // PARAMETERS FOR LINEAR ADVANCE
    if(ros::param::has("/mobile_base/max_linear_vel")) ros::param::get("/mobile_base/max_linear_vel", max_linear_vel);
    else { ROS_ERROR("profiles.->THERE'S NO PARAMETERS FOR /mobile_base/max_linear_vel"); return false; }

    if(ros::param::has("/mobile_base/speed_increment")) ros::param::get("/mobile_base/speed_increment", speed_increment);
    else { ROS_ERROR("profiles.->THERE'S NO PARAMETERS FOR /mobile_base/speed_increment"); return false; }

    if(ros::param::has("/mobile_base/linear_alpha")) ros::param::get("/mobile_base/linear_alpha", linear_alpha);
    else { ROS_ERROR("profiles.->THERE'S NO PARAMETERS FOR /mobile_base/linear_alpha"); return false; }

    // PARAMETERS FOR ANGULAR TWIST
    if(ros::param::has("/mobile_base/max_angular_vel")) ros::param::get("/mobile_base/max_angular_vel", max_angular_vel);
    else { ROS_ERROR("profiles.->THERE'S NO PARAMETERS FOR /mobile_base/max_angular_vel"); return false; }

    if(ros::param::has("/mobile_base/angular_alpha")) ros::param::get("/mobile_base/angular_alpha", angular_alpha);
    else { ROS_ERROR("profiles.->THERE'S NO PARAMETERS FOR /mobile_base/angular_alpha"); return false; }

    return true;
}

bool isRunning() {
    bool enable_movements = false;
    if(ros::param::has("/mobile_base/enable_movements")) ros::param::get("/mobile_base/enable_movements", enable_movements);
    else { ROS_ERROR("profiles.->THERE'S NO PARAMETERS FOR /mobile_base/enable_movements"); return false; }
    return enable_movements;
}

float uniformProfile(float error) {
    if (error >= 0) {
        return max_linear_vel;
    } else {
        return -max_linear_vel;
    }
}

float uniformAngularProfile(float goal) {
    if (goal >= 0) {
        return max_angular_vel;
    } else {
        return -max_angular_vel;
    }
}


float trapezoidalProfile(float curr, float goal) {
    float dir = goal / fabs(goal);
    if (current_speed < max_linear_vel) current_speed += speed_increment;

    if (curr >= goal) current_speed = 0;
    return dir * current_speed;
}

float sigmoideProfile(float angle_error, float alpha) {
    return max_angular_vel * (2 / (1 + exp( -angle_error / alpha)) - 1);
}

geometry_msgs::Twist getAngularVelocity(float goal_angle, float angle_error) {
    geometry_msgs::Twist angular_vel;
    angular_vel.linear.x = 0.0;
    angular_vel.linear.y = 0.0;
 //angular_vel.angular.z = sigmoideProfile(angle_error, angular_alpha);
    angular_vel.angular.z = uniformAngularProfile(goal_angle);

    return angular_vel;
}

geometry_msgs::Twist getLinearVelocity(float curr, float goal, float distance_error) {
    geometry_msgs::Twist linear_velocity;
    linear_velocity.linear.x = uniformProfile(distance_error);
    linear_velocity.linear.y = 0.0;
    // linear_velocity.angular.z = sigmoideProfile(angle_error, linear_alpha);
    linear_velocity.angular.z = 0.0;

    return linear_velocity;
}

geometry_msgs::Twist stop() {
    geometry_msgs::Twist move;
    move.linear.x = 0.0;
    move.linear.y = 0.0;
    move.angular.z = 0.0;

    return move;
}

geometry_msgs::TransformStamped getTFStamped(std::string parent_frame, std::string child_frame, float robot_x, float robot_y, float robot_w) {
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = parent_frame;
    transformStamped.child_frame_id = child_frame;
    transformStamped.transform.translation.x = robot_x;
    transformStamped.transform.translation.y = robot_y;
    transformStamped.transform.translation.z = 0.0;

    tf2::Quaternion quat;
    quat.setRPY(0, 0, robot_w);
    transformStamped.transform.rotation.x = quat.x();
    transformStamped.transform.rotation.y = quat.y();
    transformStamped.transform.rotation.z = quat.z();
    transformStamped.transform.rotation.w = quat.w();

    return transformStamped;
}

#endif
