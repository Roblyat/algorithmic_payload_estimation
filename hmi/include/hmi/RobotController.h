#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <thread>
#include <atomic> 
#include <mutex>
#include <condition_variable>

struct ActionParams {
    std::string pose = "";
    std::string gripper_position = "";
    int num_moves = 0;
    double max_velocity_scaling = 1.0;
    double max_acceleration_scaling = 1.0;
    double offScale_x = 0.0;
    double offScale_y = 0.0;
    double offScale_z = 0.0;
    bool sampling = false;
    bool sample_goal = false;
    bool cartesian = false;
    bool setOrientation = false;
    double dx = 0.0;
    double dy = 0.0;
    double dz = 0.0;
    int speed = 100;
    bool move_plane = false;
    bool xy_plane = false;
    bool xz_plane = false;
    bool yz_plane = false;
};


class RobotController {
public:
    RobotController(ros::NodeHandle& N, const std::string& planning_group);
    ~RobotController();

    // Enum for action types
    enum ActionType {
        EXEC_CARTESIAN,
        EXEC_RANDOM,
        EXEC_PREDEF,
        MOVE_CARTESIAN,
        MOVE_GRIPPER
    };

    void updateAction(ActionType action, const ActionParams& params);

private:
        
    ActionParams current_params;

    // Worker method to manage different actions
    void workerMethod();

    // Helper methods for movement and trajectory planning
    void setPlanningGroup(const std::string& group_name);
    std::pair<double, double> setScalingValues();
    void setGoalTarget(geometry_msgs::Pose& target_pose);
    void setRandomOrientation(geometry_msgs::Pose& target_pose);
    bool isTrajectoryTimeIncreasing(moveit_msgs::RobotTrajectory &trajectory);
    bool computeCartesianPath(std::vector<geometry_msgs::Pose>& waypoints, moveit_msgs::RobotTrajectory& trajectory_msg);
    void planExecute(const moveit_msgs::RobotTrajectory& trajectory_msg, int trajectory_number);
    void moveInPlane(geometry_msgs::Pose& target_pose);

    // Core movement methods
    bool execCartesian();

    bool execRandom();

    bool moveCartesian();

    bool execPreDef();
    
    bool controlGripper();

    // ROS NodeHandle and MoveIt interfaces
    ros::NodeHandle nh;
    moveit::planning_interface::MoveGroupInterface move_group_interface;
    std::string current_planning_group_;

    // Threading and control flags
    std::mutex move_group_mutex;
    std::thread worker_thread;
    std::atomic<bool> worker_running;
    std::atomic<ActionType> current_action;
    std::atomic<bool> action_ready;
    std::atomic<bool> random_move_running;
    std::atomic<bool> cartesian_running; 
};

#endif  // ROBOT_CONTROLLER_H