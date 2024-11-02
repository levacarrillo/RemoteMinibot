#ifndef PROFILE_H
#define PROFILE_H

#include <iostream>
#include <geometry_msgs/Twist.h>

float current_speed;

float max_linear_speed;
float speed_increment;
float linear_alpha;

float max_angular_vel;
float angular_alpha;


bool setParameters() {
    // PARAMETERS FOR LINEAR ADVANCE
    if(ros::param::has("max_linear_speed")) ros::param::get("max_linear_speed", max_linear_speed);
    else { ROS_ERROR("There's no parameter for max_linear_speed"); return false; }

    if(ros::param::has("speed_increment")) ros::param::get("speed_increment", speed_increment);
    else { ROS_ERROR("There's no parameter for speed_increment"); return false; }

    if(ros::param::has("linear_alpha")) ros::param::get("linear_alpha", linear_alpha);
    else { ROS_ERROR("There's no parameter for linear_alpha"); return false; }

    // PARAMETERS FOR ANGULAR TWIST
    if(ros::param::has("max_angular_vel")) ros::param::get("max_angular_vel", max_angular_vel);
    else { ROS_ERROR("There's no parameter for max_angular_vel"); return false; }

    if(ros::param::has("angular_alpha")) ros::param::get("angular_alpha", angular_alpha);
    else { ROS_ERROR("There's no parameter for angular_alpha"); return false; }

    return true;
}

bool isRunning() {
    bool run_algorithm = false;
    if(ros::param::has("/run_algorithm")) ros::param::get("run_algorithm", run_algorithm);
    else { ROS_ERROR("There's no parameter for run_algorithm"); return false; }
    return run_algorithm;
}

float trapezoidalProfile(float curr, float goal) {
    float dir = goal / fabs(goal);
    if (current_speed < max_linear_speed) current_speed += speed_increment;

    if (curr >= goal) current_speed = 0;
    return dir * current_speed;
}

float sigmoideProfile(float angle_error, float alpha) {
    return max_angular_vel * (2 / (1 + exp( -angle_error / alpha)) - 1);
}

geometry_msgs::Twist getAngularVelocity(float angle_error) {
    geometry_msgs::Twist angular_vel;
    angular_vel.linear.x = 0.0;
    angular_vel.linear.y = 0.0;
    angular_vel.angular.z = sigmoideProfile(angle_error, angular_alpha);

    return angular_vel;
}

geometry_msgs::Twist getLinearVelocity(float curr, float goal, float angle_error) {
    geometry_msgs::Twist linear_velocity;
    linear_velocity.linear.x = trapezoidalProfile(curr, goal);
    linear_velocity.linear.y = 0.0;
    linear_velocity.angular.z = sigmoideProfile(angle_error, linear_alpha);

    return linear_velocity;
}

geometry_msgs::Twist stop() {
    geometry_msgs::Twist move;
    move.linear.x = 0.0;
    move.linear.y = 0.0;
    move.angular.z = 0.0;

    return move;
}

#endif
