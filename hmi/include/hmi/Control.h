#ifndef CONTROL_H
#define CONTROL_H

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include "imgui.h"
#include <iostream>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <geometry_msgs/PoseArray.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "hmi/trajectory_msg.h"

class Control {
public:
    Control(ros::NodeHandle& N);
    void render();

private:
    void moveInit(const std::string& planning_group); // Method to move to initial position
    void controlGripper(const std::string& action);

    // Functions for linear movements
    void moveLinXPos();
    void moveLinXNeg();
    void moveLinYPos();
    void moveLinYNeg();
    void moveLinZPos();
    void moveLinZNeg();

    // Other functionalities
    void moveRobotToPose(const std::string& planning_group, const geometry_msgs::Pose& pose);
    void poseCallback(const geometry_msgs::PoseArray::ConstPtr &msg);
    geometry_msgs::Pose handlePose(int index, const geometry_msgs::PoseArray& poses);

    void accPos();  // Increase acceleration
    void accNeg();  // Decrease acceleration

    // MoveGroup constraints
    char planning_group[128] = "manipulator"; 
    moveit_msgs::RobotTrajectory trajectory;
    std::vector<geometry_msgs::Pose> waypoints;
    int speed = 1;  // Speed factor

    // ROS variables
    ros::NodeHandle nh;
    ros::Publisher trajPub;
    ros::Subscriber poseSub;
    geometry_msgs::PoseArray target_poses;

    double distance = 0.1;  // Movement distance
    double acceleration_scaling_factor = 0.1;  // Scaling factor for acceleration
    double velocity_scaling_factor = 0.1;  // Scaling factor for velocity

    // Button sizes
    ImVec2 button_30x10; 
    ImVec2 button_5x5; 
};

#endif // CONTROL_H
