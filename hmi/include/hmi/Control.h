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


class Control {
public:
    Control(ros::NodeHandle& N);
    void render();
private:
    void setupButtonCallback();
    void beginExternalControlButtonCallback();
    void defaultQUANTECCallback();
    void defaultUR10Callback();
    void moveRobotToHome(const std::string& planning_group, const std::string& target_pose);
    void poseCallback(const geometry_msgs::PoseArray::ConstPtr &msg);
    void moveRobotToPose(const std::string& planning_group, const geometry_msgs::Pose& pose);
    void moveLIN(const std::string& planning_group, const std::string &referenceLink);
    geometry_msgs::Pose handlePose(int index, const geometry_msgs::PoseArray& poses);

    ros::NodeHandle nh;

    ros::Publisher indexControlURProgramPub;
    ros::Subscriber poseSub;
    geometry_msgs::PoseArray target_poses;
    double distance = 0.1;  


    int setupValue = 1, externalControlValue = 2, defaultValue = 0;
};

#endif // CONTROL_H