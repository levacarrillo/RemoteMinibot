#include <ros/ros.h>
#include <../utilities/structures.h>
#include <motion_planner/motion_planner.h>
#include <motion_planner/motion_planner_utilities.h>
#include <../state_machines/user_sm.h>
#include <../state_machines/light_follower.h>
#include <../state_machines/sm_destination.h>
#include <../state_machines/sm_avoid_obstacles.h>
#include <../state_machines/sm_avoidance_destination.h>


int main(int argc, char* argv[]) {
    ros::init(argc, argv, "motion_planner_node");
    ros::NodeHandle nh;
    ros::Rate rate(20);

    bool enable_movements = false;

    float max_intensity;
    float* light_readings;
    int light_destination;
    int obstacles_detected;
    float* lidar_readings;

    int next_state = 1;

    movement movement;
    MotionPlanner robot(nh);


    while(ros::ok()) {
        enable_movements = robot.is_running();
        Behaviors behavior = robot.get_behavior();

        if (enable_movements) {
            light_readings = robot.get_light_readings();
            max_intensity = robot.get_max_intensity();
            light_destination = get_light_direction(light_readings);
            lidar_readings = robot.get_lidar_readings();
            obstacles_detected = obstacle_detection(lidar_readings, 3, 0.2);

            // std::cout << "motion_planner.-> light_readings: [";
            // for(int i=0; i<8; i++) {
            //     std::cout << light_readings[i] << ", ";
            // }
            // std::cout << "]" << std::endl;

            switch(behavior) {
                case NONE:
                    enable_movements = true;
                break;
                case LIGHT_FOLLOWER:
                    enable_movements = !light_follower(max_intensity, light_readings, &movement, robot.get_max_advance(), robot.get_threshold_follower());
                break;
                case SM_DESTINATION:
                    enable_movements = !sm_destination(max_intensity, light_destination, &movement, &next_state, robot.get_max_advance(), robot.get_max_turn_angle());
                break;
                case SM_AVOID_OBSTACLES:
                    enable_movements = !sm_avoid_obstacles(lidar_readings, 3, obstacles_detected, &movement, &next_state, robot.get_max_advance(), robot.get_max_turn_angle());
                break;
                case SM_AVOIDANCE_DESTINATION:
                    enable_movements = !sm_avoidance_destination(lidar_readings, 3, max_intensity, light_destination, obstacles_detected, &movement, &next_state, robot.get_max_advance(), robot.get_max_turn_angle());
                break;
                case USER_SM:
                    enable_movements = !user_sm(max_intensity, light_readings, lidar_readings, 3, 0.3, light_destination, obstacles_detected, &movement, &next_state, robot.get_max_advance(), robot.get_max_turn_angle());
                break;
                default:
                    std::cout << " *************** motion_planner.->  NO BEHAVIOR DEFINED *************** " << std::endl;
                    movement.twist = 0.0;
                    movement.advance = 0.0;
                break;
            }

            if (!enable_movements) robot.stop_algorithm();
            
            if(behavior != NONE) {
                std::cout << "\n \n  MOTION PLANNER \n____________________________\n" << std::endl;
                std::cout << "motion_planner.-> BEHAVIOR SELECTED->" << behavior << std::endl;
                // std::cout << "Light" << std::endl;
                // std::cout << "Robot: " << std::endl;
                // std::cout << "Step" << std::endl;
                // std::cout << "motion_planner.-> Movement: twist: " << movement.twist << "\tadvance: " << movement.advance << "\n" << std::endl;
                robot.move_to_pose(movement.twist, movement.advance);
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}