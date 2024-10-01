#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>

// RobotController class handles robot movement operations
class RobotController {
public:
    // Constructor
    RobotController(ros::NodeHandle& N, const std::string& planning_group);

    // Method for Cartesian movement
    void moveCartesian(double dx, double dy, double dz, int speed);

    void execPreDef(std::string &pose);
    void moveRandom(int num_moves, int max_valid_attempts, double max_velocity_scaling, 
        double max_acceleration_scaling, int planning_attempts, double planning_time);

    void controlGripper(const std::string &position, double speed);

    // Function to dynamically set the planning group
    void setPlanningGroup(const std::string& group_name);

    std::mutex move_group_mutex;  // Mutex to protect shared resources

private:
    ros::NodeHandle nh;
    moveit::planning_interface::MoveGroupInterface move_group_interface;
    std::string current_planning_group_;
};

#endif // ROBOT_CONTROLLER_H
