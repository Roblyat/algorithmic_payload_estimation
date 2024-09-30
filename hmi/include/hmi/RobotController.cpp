#include "RobotController.h"

// Constructor
RobotController::RobotController(ros::NodeHandle& N, const std::string& planning_group)
    : nh(N), move_group_interface(planning_group) {}


// Method for Cartesian movement
void RobotController::moveCartesian(double dx, double dy, double dz, int speed) {
    geometry_msgs::PoseStamped current_pose = move_group_interface.getCurrentPose();
    geometry_msgs::Pose target = current_pose.pose;
    target.position.x += dx * speed * 0.001;
    target.position.y += dy * speed * 0.001;
    target.position.z += dz * speed * 0.001;

    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(target);

    moveit_msgs::RobotTrajectory trajectory;
    const double eef_step = 0.01;
    const double jump_threshold = 0.0;
    double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    move_group_interface.asyncExecute(trajectory);
}

void RobotController::movePreDef(std::string &pose) {
    // Set the target pose to the home position for the robot
    move_group_interface.setNamedTarget(pose);

    // Execute the motion asynchronously (non-blocking)
    moveit::planning_interface::MoveItErrorCode result = move_group_interface.asyncMove(); 

    // Check if the command was successfully sent
    if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_INFO("Robot is moving to the home position asynchronously.");
    } else {
        ROS_WARN("Failed to move to the home position.");
    }
}