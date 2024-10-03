#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <thread>
#include <atomic> 
#include <mutex>
#include <condition_variable>

// RobotController class handles robot movement operations
class RobotController {
public:
    // Constructor
    RobotController(ros::NodeHandle& N, const std::string& planning_group);
    // Destructor
    ~RobotController();

    // Method for Cartesian movement
    void moveCartesian(double dx, double dy, double dz, int speed);

    void execPreDef(std::string &pose);
    
    //deprecated
    void moveRandom(int num_moves, int max_valid_attempts, double max_velocity_scaling, 
        double max_acceleration_scaling, int planning_attempts, double planning_time);
    //
    
    void executeJerkTrajectory(int num_moves, double max_velocity_scaling, double max_acceleration_scaling, 
        double offScale_x, double offScale_y, double offScale_z);

    void controlGripper(const std::string &position, double speed);

    //thread handling methods;
    void startJerkTrajectory(int num_moves, double max_velocity_scaling, double max_acceleration_scaling,
        double offScale_x, double offScale_y, double offScale_z);

    void stopJerkTrajectory();

    void startRandomMove(int num_moves, int max_valid_attempts, double max_velocity_scaling,
        double max_acceleration_scaling, int planning_attempts, double planning_time);
    
    void stopRandomMove();

    // Function to dynamically set the planning group
    void setPlanningGroup(const std::string& group_name);
   
private:
    ros::NodeHandle nh;
    moveit::planning_interface::MoveGroupInterface move_group_interface;
    std::string current_planning_group_;


    //thread handling variables
    std::mutex move_group_mutex;
    std::thread jerk_thread;
    std::thread random_move_thread;

    std::atomic<bool> jerk_running;
    std::atomic<bool> random_move_running;

    std::condition_variable jerk_cv;
    std::condition_variable random_move_cv;
};

#endif // ROBOT_CONTROLLER_H
