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

void RobotController::controlGripper(const std::string &position, double speed) {
    setPlanningGroup("gripper");

    // Set the target position for the gripper (either "open" or "closed")
    move_group_interface.setNamedTarget(position);

    // Set gripper speed
    move_group_interface.setMaxVelocityScalingFactor(speed);  // Adjust the speed (value between 0 and 1)

    // Plan and execute the movement
    moveit::planning_interface::MoveItErrorCode result = move_group_interface.asyncMove();

    // Provide feedback based on the execution result
    if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_INFO("Gripper moved to the %s position at speed %.2f.", position.c_str(), speed);
    } else {
        ROS_WARN("Failed to move the gripper to the %s position.", position.c_str());
    }
}

void RobotController::moveRandom(int num_moves, int max_valid_attempts, double max_velocity_scaling, double max_acceleration_scaling, int planning_attempts, double planning_time) {
    
    std::lock_guard<std::mutex> lock(move_group_mutex);
    
    setPlanningGroup("manipulator");

    // Set the planning parameters: attempts and planning time
    move_group_interface.setNumPlanningAttempts(planning_attempts);
    move_group_interface.setPlanningTime(planning_time);

    // Loop to execute random moves
    for (int i = 0; i < num_moves; ++i) {
        bool success = false;
        int attempts = 0;

        // Try to plan a valid random goal
        while (!success && attempts < max_valid_attempts) {
            
             move_group_interface.setStartStateToCurrentState();

            // Generate random valid pose
            geometry_msgs::Pose target_pose = move_group_interface.getRandomPose("sensor_robotiq_ft_frame_id").pose;

            // Set this as the target pose
            move_group_interface.setPoseTarget(target_pose);

            // Randomize velocity and acceleration scaling, but cap them to the provided max values
            double velocity_scaling = 0.1 + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (max_velocity_scaling - 0.1))); 
            double acceleration_scaling = 0.1 + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (max_acceleration_scaling - 0.1)));
            move_group_interface.setMaxVelocityScalingFactor(velocity_scaling);
            move_group_interface.setMaxAccelerationScalingFactor(acceleration_scaling);

            // Plan the motion
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            success = (move_group_interface.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

            // If the plan was successful, execute it asynchronously
            if (success) {
                // plan.simplify();  // Simplifies the trajectory if applicable
                ROS_INFO("Successfully planned random pose %d with velocity scaling %.2f and acceleration scaling %.2f", i + 1, velocity_scaling, acceleration_scaling);
                moveit::planning_interface::MoveItErrorCode result = move_group_interface.asyncExecute(plan);

                // Check if execution was successful
                if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                    ROS_INFO("Successfully executed random pose %d.", i + 1);
                    break;  // Exit the retry loop, move to the next random motion
                } else {
                    ROS_WARN("Failed to execute random pose %d, retrying...", i + 1);
                    success = false;  // Reset success for reattempt
                }
            } else {
                ROS_WARN("Failed to plan random pose %d, retrying...", i + 1);
            }

            attempts++;
        }

        // If after max attempts, we still can't plan, log it
        if (!success) {
            ROS_ERROR("Failed to plan a random valid pose after %d attempts.", max_valid_attempts);
        }
    }
}


void RobotController::executeJerkTrajectory(int num_moves, double max_velocity_scaling, double max_acceleration_scaling) {
    
    std::lock_guard<std::mutex> lock(move_group_mutex);

    setPlanningGroup("manipulator");

    for (int i = 0; i < num_moves; ++i) {
        
        // Get current pose
        geometry_msgs::PoseStamped current_pose = move_group_interface.getCurrentPose("sensor_robotiq_ft_frame_id");
        geometry_msgs::Pose target_pose = current_pose.pose;

        // Generate small random movements (short trajectory)
        double random_delta_x = (static_cast<double>(rand()) / RAND_MAX - 0.5) * 0.3;  // Random small movement in X
        double random_delta_y = (static_cast<double>(rand()) / RAND_MAX - 0.5) * 0.3;  // Random small movement in Y
        double random_delta_z = (static_cast<double>(rand()) / RAND_MAX - 0.5) * 0.3;  // Random small movement in Z

        // Update target position with random deltas
        target_pose.position.x += random_delta_x;
        target_pose.position.y += random_delta_y;
        target_pose.position.z += random_delta_z;

        // Ensure start state matches current robot state
        move_group_interface.setStartStateToCurrentState();

        // Set the target pose
        move_group_interface.setPoseTarget(target_pose);

        // Set maximum velocity and acceleration scaling
        move_group_interface.setMaxVelocityScalingFactor(max_velocity_scaling);
        move_group_interface.setMaxAccelerationScalingFactor(max_acceleration_scaling);

        // Plan the trajectory
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_interface.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        // Execute the trajectory if planning was successful
        if (success) {
            ROS_ERROR("Successfully planned random short trajectory %d with small deltas.", i + 1);
            moveit::planning_interface::MoveItErrorCode result = move_group_interface.asyncExecute(plan);
            
            // Check if the execution was successful asynchronously
            if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                ROS_ERROR("Successfully executed short trajectory %d.", i + 1);

                // Logic to wait for completion without creating a local variable for the action client
                while (!move_group_interface.getMoveGroupClient().getState().isDone()) {
                    ros::Duration(2.0).sleep();  // Wait for a short time before checking again
                }

                // Check if execution was successful
                if (move_group_interface.getMoveGroupClient().getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                    ROS_INFO("Execution of trajectory %d completed successfully.", i + 1);
                } else {
                    ROS_WARN("Execution of trajectory %d failed or was preempted.", i + 1);
                }

            } else {
                ROS_WARN("Failed to execute short trajectory %d, retrying...", i + 1);
            }
        } else {
            ROS_WARN("Failed to plan short trajectory %d, skipping...", i + 1);
        }
    }
}

