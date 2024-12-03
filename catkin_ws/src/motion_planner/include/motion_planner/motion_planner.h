#ifndef MOTION_PLANNER_UTILITIES_H
#define MOTION_PLANNER_UTILITIES_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <hardware/LightReadings.h>
#include <mobile_base/MoveMinibot.h>

class MotionPlanner {
    private:
        ros::NodeHandle &nh_;
        ros::Subscriber lidar_sub;
        ros::ServiceClient movement_client, light_client;
        std::string behavior;
        bool enable_movements;
        float max_advance_distance;
        float max_turn_angle;
        float light_readings[8];
        float max_intensity;
        float threshold_follower;

        float lidar_readings[3];

        void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
            for (int i=0; i<3; i++) {
                lidar_readings[i] = float(msg->ranges[i]);
            }
        }

    public:
        MotionPlanner(ros::NodeHandle &nh) : nh_(nh) {
            movement_client = nh_.serviceClient<mobile_base::MoveMinibot>("/mobile_base/move_to_pose");
            light_client = nh_.serviceClient<hardware::LightReadings>("/hardware/light_readings");
            lidar_sub = nh_.subscribe("/hardware/scan", 10, &MotionPlanner::laserCallback, this);
        }
        
        float* get_lidar_readings() {
            return lidar_readings;
        }

        void move_to_pose(float theta, float advance) {
            mobile_base::MoveMinibot srv;
            srv.request.theta    = theta;
            srv.request.distance = advance;
            if (movement_client.call(srv)) {
                if(srv.response.done) std::cout << "motion_planner.-> ROBOT MOVEMENT DONE" << std::endl;
                else ROS_ERROR("motion_planner.->FAILED TO ROBOT MOVEMENT");
            } else {
                ROS_ERROR("motion_planner.->FAILED TO CALL SERVICE /mobile_base/move_to_pose");
            }
        }


        float* get_light_readings() {
            hardware::LightReadings srv;
            if (light_client.call(srv)) {
                max_intensity = srv.response.max_intensity;
                // std::cout << "motion_planner.-> srv.response.light_readings.size(): " << srv.response.light_readings.size() << std::endl; 
                // std::cout << "[";
                for (size_t i=0; i<srv.response.light_readings.size(); i++) {
                    // std::cout << light_readings[i] << ", ";
                    light_readings[i] = srv.response.light_readings[i];
                }
                // std::cout << "]" << std::endl;
            } else {
                ROS_ERROR("motion_planner.->FAILED TO CALL SERVICE /hardware/light_readings");
            }
            return light_readings;
        }

        float get_max_intensity() {
            return max_intensity;
        }


        Behaviors get_behavior() {
            if (!nh_.getParam("/motion_planner/behavior", behavior)) {
                ROS_ERROR("motion_planner.->FAILED TO GET PARAMETER /motion_planner/behavior");
                return NOT_DEFINED;
            }
            if (behavior == "none")                return NONE;
            if (behavior == "user_sm")             return USER_SM;
            if (behavior == "sm_destination")      return SM_DESTINATION;
            if (behavior == "light_follower")      return LIGHT_FOLLOWER;
            if (behavior == "sm_avoid_obstacles")  return SM_AVOID_OBSTACLES;
            if (behavior == "sm_avoidance_destination") return SM_AVOIDANCE_DESTINATION;

            return NOT_DEFINED;
        }

        float get_max_advance() {
            if (!nh_.getParam("/motion_planner/max_advance_distance", max_advance_distance)) {
                ROS_ERROR("motion_planner.->FAILED TO GET PARAMETER /motion_planner/max_advance_distance OF ROBOT");
            }
            return max_advance_distance;
        }
        
        float get_max_turn_angle() {
            if (!nh_.getParam("/motion_planner/max_turn_angle", max_turn_angle)) {
                ROS_ERROR("motion_planner.->FAILED TO GET PARAMETER /motion_planner/max_turn_angle OF ROBOT");
            }
            return max_turn_angle;
        }

        bool is_running() {
            if (!nh_.getParam("/mobile_base/enable_movements", enable_movements)) {
                ROS_ERROR("motion_planner.->FAILED TO GET PARAMETER /mobile_base/enable_movements");
                return false;
            }
            return enable_movements;
        }

        void stop_algorithm() {
            nh_.setParam("/mobile_base/enable_movements", false);
            ros::Duration(1);
        }

        float get_threshold_follower() {
            if (!nh_.getParam("/motion_planner/threshold_follower", threshold_follower)) {
                ROS_ERROR("motion_planner.->FAILED TO GET PARAMETER /motion_planner/threshold_follower PARAMETER");
            }
            return threshold_follower;
        }
};

#endif
