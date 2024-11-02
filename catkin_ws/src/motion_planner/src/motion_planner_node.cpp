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

    bool run_algorithm = false;

    float max_intensity;
    float* light_readings;
    int light_destination;
    int obstacles_detected;
    float* lidar_readings;

    int next_state = 1;

    movement movement;
    MotionPlanner node(nh);


    while(ros::ok()) {
        run_algorithm = node.is_running();
        Behaviors behavior = node.get_behavior();

        if (run_algorithm) {
            light_readings = node.get_light_readings();
            max_intensity = node.get_max_intensity();
            light_destination = get_light_direction(light_readings);
            lidar_readings = node.get_lidar_readings();
            obstacles_detected = obstacle_detection(lidar_readings, 3, 0.2);

            std::cout << "BEHAVIOR SELECTED->" << behavior << std::endl;
            switch(behavior) {
                case LIGHT_FOLLOWER:
                    run_algorithm = !light_follower(max_intensity, light_readings, &movement, node.get_max_advance());
                break;
                case SM_DESTINATION:
                    run_algorithm = !sm_destination(max_intensity, light_destination, &movement, &next_state, node.get_max_advance(), node.get_max_turn_angle());
                break;
                case SM_AVOID_OBSTACLES:
                    run_algorithm = !sm_avoid_obstacles(lidar_readings, 3, obstacles_detected, &movement, &next_state, node.get_max_advance(), node.get_max_turn_angle());
                break;
                case SM_AVOIDANCE_DESTINATION:
                    run_algorithm = !sm_avoidance_destination(lidar_readings, 3, max_intensity, light_destination, obstacles_detected, &movement, &next_state, node.get_max_advance(), node.get_max_turn_angle());
                break;
                case USER_SM:
                    run_algorithm = !user_sm(max_intensity, light_readings, lidar_readings, 3, 0.3, light_destination, obstacles_detected, &movement, &next_state, node.get_max_advance(), node.get_max_turn_angle());
                break;
                default:
                    std::cout << " *************** NO BEHAVIOR DEFINED *************** " << std::endl;
                    movement.twist = 0.0;
                    movement.advance = 0.0;
                break;
            }

            if (!run_algorithm) node.stop_algorithm();
            
            // std::cout << "[";
            // for (size_t i=0; i<8; i++) {
            //     std::cout << lidar_readings[i] << ",";
            // }
            // std::cout << "]" << std::endl;

            std::cout << "\n \n  MOTION PLANNER \n____________________________\n" << std::endl;
            // std::cout << "Light" << std::endl;
            // std::cout << "Robot: " << std::endl;
            // std::cout << "Step" << std::endl;
            std::cout << "Movement: twist: " << movement.twist << " advance: " << movement.advance << "\n" << std::endl;

        
            node.move_robot(movement.twist, movement.advance);
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}