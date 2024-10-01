#include "RobotController.h"

// Constructor
RobotController::RobotController(ros::NodeHandle& N, const std::string& planning_group)
    : nh(N), move_group_interface(planning_group), current_planning_group_(planning_group) {}

// Method to dynamically set the planning group
void RobotController::setPlanningGroup(const std::string& group_name) {
    // Update the current planning group
    current_planning_group_ = group_name;

    // Reinitialize the MoveGroupInterface with the new group
    move_group_interface = moveit::planning_interface::MoveGroupInterface(group_name);
}

// Method for Cartesian movement
void RobotController::moveCartesian(double dx, double dy, double dz, int speed) {

    setPlanningGroup("manipulator");

    geometry_msgs::PoseStamped current_pose = move_group_interface.getCurrentPose();
    geometry_msgs::Pose target = current_pose.pose;
    
    // Adjust target position based on the input deltas (dx, dy, dz)
    target.position.x += dx * speed * 0.001;
    target.position.y += dy * speed * 0.001;
    target.position.z += dz * speed * 0.001;

    // Create a list of waypoints (just the target in this case)
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(target);

    // Define variables for planning
    moveit_msgs::RobotTrajectory trajectory;
    const double eef_step = 0.01;  // Step size
    const double jump_threshold = 0.0;  // No jump threshold for now

    // Compute the Cartesian path
    double fraction = move_group_interface.computeCartesianPath(
        waypoints, eef_step, jump_threshold, trajectory, true /* collision checking */);

    // Check if the path computation was successful
    if (fraction < 0.95) {
        ROS_WARN("Unable to plan the entire Cartesian path. Only %.2f%% of the path was planned.", fraction * 100.0);
        return;
    }

    // Create a plan object to hold the trajectory
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    // Execute the planned trajectory
    moveit::planning_interface::MoveItErrorCode result = move_group_interface.asyncExecute(plan);

    // Provide feedback based on the execution result
    if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_INFO("Cartesian path executed successfully.");
    } else {
        ROS_WARN("Failed to execute the Cartesian path.");
    }
}


void RobotController::execPreDef(std::string &pose) {

    setPlanningGroup("manipulator");

    // Set the target pose to the home position for the robot
    move_group_interface.setNamedTarget(pose);

    // Configure collision checking and set parameters for planning
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success) {
        moveit::planning_interface::MoveItErrorCode result = move_group_interface.asyncExecute(plan);
        if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            ROS_INFO("Robot is moving to the %s position asynchronously.", pose.c_str());
        } else {
            ROS_WARN("Failed to move to the %s position.", pose.c_str());
        }
    } else {
        ROS_WARN("Planning to %s failed due to collision or other constraints.", pose.c_str());
    }
}

void RobotController::controlGripper(const std::string &position) {

    setPlanningGroup("gripper");

    // Set the target position for the gripper (either "open" or "closed")
    move_group_interface.setNamedTarget(position);

    // Plan and execute the movement
    moveit::planning_interface::MoveItErrorCode result = move_group_interface.asyncMove();

    // Provide feedback based on the execution result
    if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_INFO("Gripper moved to the %s position.", position.c_str());
    } else {
        ROS_WARN("Failed to move the gripper to the %s position.", position.c_str());
    }
}
