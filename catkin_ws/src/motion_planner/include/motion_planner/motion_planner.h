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
        bool run_algorithm;
        float max_advance;
        float max_turn_angle;
        std::string behavior;
        float light_readings[8];
        float max_intensity;

        float lidar_readings[3];

        void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
            for (int i=0; i<3; i++) {
                lidar_readings[i] = float(msg->ranges[i]);
            }
        }

    public:
        MotionPlanner(ros::NodeHandle &nh) : nh_(nh) {
            movement_client = nh_.serviceClient<mobile_base::MoveMinibot>("move_robot");
            light_client = nh_.serviceClient<hardware::LightReadings>("light_readings");
            lidar_sub = nh_.subscribe("/scan", 10, &MotionPlanner::laserCallback, this);
        }
        
        float* get_lidar_readings() {
            return lidar_readings;
        }

        void move_robot(float theta, float advance) {
            mobile_base::MoveMinibot srv;
            srv.request.theta    = theta;
            srv.request.distance = advance;
            if (movement_client.call(srv)) {
                if(srv.response.done) std::cout << "ROBOT MOVEMENT DONE" << std::endl;
                else ROS_ERROR("FAILED TO ROBOT MOVEMENT");
            } else {
                ROS_ERROR("Failed to call service /move_robot");
            }
        }


        float* get_light_readings() {
            hardware::LightReadings srv;
            if (light_client.call(srv)) {
                max_intensity = srv.response.max_intensity;
                for (size_t i=0; i<srv.response.light_readings.size(); i++) {
                    light_readings[i] = srv.response.light_readings[i];
                }
            } else {
                ROS_ERROR("FAILED TO CALL SERVICE /light_readings");
            }
            return light_readings;
        }

        float get_max_intensity() {
            return max_intensity;
        }


        Behaviors get_behavior() {
            if (!nh_.getParam("/behavior", behavior)) {
                ROS_ERROR("FAILED TO GET PARAMETER /behavior");
                return NOT_DEFINED;
            }

            if (behavior == "user_sm")             return USER_SM;
            if (behavior == "sm_destination")      return SM_DESTINATION;
            if (behavior == "light_follower")      return LIGHT_FOLLOWER;
            if (behavior == "sm_avoid_obstacles")  return SM_AVOID_OBSTACLES;
            if (behavior == "sm_avoidance_destination") return SM_AVOIDANCE_DESTINATION;

            return NOT_DEFINED;
        }

        float get_max_advance() {
            if (!nh_.getParam("/max_advance", max_advance)) {
                ROS_ERROR("FAILED TO GET PARAMETER /max_advance OF ROBOT");
            }
            return max_advance;
        }
        
        float get_max_turn_angle() {
            if (!nh_.getParam("/max_turn_angle", max_turn_angle)) {
                ROS_ERROR("FAILED TO GET PARAMETER /max_turn_angle OF ROBOT");
            }
            return max_turn_angle;
        }

        bool is_running() {
            if (!nh_.getParam("/run_algorithm", run_algorithm)) {
                ROS_ERROR("FAILED TO GET PARAMETER /run_algorithm");
                return false;
            }
            return run_algorithm;
        }

        void stop_algorithm() {
            nh_.setParam("/run_algorithm", false);
            ros::Duration(1);
        }
};

#endif
