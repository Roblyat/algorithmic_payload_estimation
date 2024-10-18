#include "RobotController.h"
#include <cstdlib>

// Constructor
RobotController::RobotController(ros::NodeHandle& N, const std::string& planning_group)
    : nh(N), move_group_interface(planning_group), current_planning_group_(planning_group), 
        jerk_running(false), random_move_running(false) {}

// Destructor
RobotController::~RobotController() {
    stopJerkTrajectory();  // Ensures that jerk thread is stopped
    stopRandomMove();      // Ensures that random move thread is stopped
}


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

// Deprecated method: need to be switched to sync execution, is executed on seperate thread
void RobotController::moveRandom(int num_moves, int max_valid_attempts, double max_velocity_scaling, double max_acceleration_scaling, int planning_attempts, double planning_time) {
    
    std::lock_guard<std::mutex> lock(move_group_mutex);
    
    setPlanningGroup("manipulator");

    // Set the planning parameters: attempts and planning time
    move_group_interface.setNumPlanningAttempts(planning_attempts);
    move_group_interface.setPlanningTime(planning_time);

    // Loop to execute random moves
    for (int i = 0; i < num_moves; ++i) { 
        
        //check if thread is running or stopped by user
        if (!random_move_running.load()) {
            ROS_INFO("Random move stopped early.");
            return;
        }

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

//  Deprecated method:
void RobotController::executeJerkTrajectory(int num_moves, double max_velocity_scaling, double max_acceleration_scaling, double offScale_x, double offScale_y, double offScale_z) {
    
    std::lock_guard<std::mutex> lock(move_group_mutex);

    setPlanningGroup("manipulator");

    //check if thread is running or stopped by user
    for (int i = 0; i < num_moves; ++i) {
        if (!jerk_running.load()) {
            ROS_INFO("Jerk trajectory stopped early.");
            return;
        }
        
        // Get current pose
        geometry_msgs::PoseStamped current_pose = move_group_interface.getCurrentPose("sensor_robotiq_ft_frame_id");
        geometry_msgs::Pose target_pose = current_pose.pose;

        // Generate small random movements (short trajectory)
        if (offScale_x != 0) {
            double random_delta_x = (static_cast<double>(rand()) / RAND_MAX - 0.5) * offScale_x;  // Random small movement in X
            target_pose.position.x += random_delta_x;
        }

        if (offScale_y != 0) {
            double random_delta_y = (static_cast<double>(rand()) / RAND_MAX - 0.5) * offScale_y;  // Random small movement in Y
            target_pose.position.y += random_delta_y;
        }

        if (offScale_z != 0) {
            double random_delta_z = (static_cast<double>(rand()) / RAND_MAX - 0.5) * offScale_z;  // Random small movement in Z
            target_pose.position.z += random_delta_z;
        }

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

        if (success) {
            ROS_ERROR("Successfully planned random short trajectory %d with small deltas.", i + 1);
            
            // Execute the trajectory synchronously
            moveit::planning_interface::MoveItErrorCode result = move_group_interface.execute(plan);
            
            if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                ROS_ERROR("Successfully executed short trajectory %d.", i + 1);
            } else {
                ROS_WARN("Failed to execute short trajectory %d.", i + 1);
            }
        } else {
            ROS_WARN("Failed to plan short trajectory %d, skipping...", i + 1);
        }
    }
}

//move rand cartesian
void RobotController::executeCartesianJerkTrajectory(int num_moves, double max_velocity_scaling, double max_acceleration_scaling, double offScale_x, double offScale_y, double offScale_z) {

    std::lock_guard<std::mutex> lock(move_group_mutex);

    setPlanningGroup("manipulator");

    for (int i = 0; i < num_moves; ++i) {
        if (!cartesian_running.load()) {
            ROS_INFO("Cartesian jerk trajectory stopped early.");
            return;
        }
        
        // Get the current pose
        geometry_msgs::PoseStamped current_pose = move_group_interface.getCurrentPose("sensor_robotiq_ft_frame_id");
        geometry_msgs::Pose target_pose = current_pose.pose;

        // Generate small random movements (short trajectory)
        if (offScale_x != 0) {
            double random_delta_x = (static_cast<double>(rand()) / RAND_MAX - 0.5) * offScale_x;  // Random small movement in X
            target_pose.position.x += random_delta_x;
        }

        if (offScale_y != 0) {
            double random_delta_y = (static_cast<double>(rand()) / RAND_MAX - 0.5) * offScale_y;  // Random small movement in Y
            target_pose.position.y += random_delta_y;
        }

        if (offScale_z != 0) {
            double random_delta_z = (static_cast<double>(rand()) / RAND_MAX - 0.5) * offScale_z;  // Random small movement in Z
            target_pose.position.z += random_delta_z;
        }

        // Ensure the start state matches the current robot state
        move_group_interface.setStartStateToCurrentState();

        // Cartesian path planning using waypoints
        std::vector<geometry_msgs::Pose> waypoints;
        waypoints.push_back(current_pose.pose);  // Start from the current pose
        waypoints.push_back(target_pose);  // Move to the target pose

        // Plan a Cartesian path
        moveit_msgs::RobotTrajectory trajectory_msg;
        const double eef_step = 0.01;  // Step size in meters
        const double jump_threshold = 0.0;  // Disable jump threshold

        double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory_msg);

        if (fraction > 0.95) {
            ROS_INFO("Successfully computed Cartesian path %d.", i + 1);

            // Convert moveit_msgs::RobotTrajectory to robot_trajectory::RobotTrajectory
            robot_trajectory::RobotTrajectory robot_trajectory(move_group_interface.getRobotModel(), "manipulator");
            robot_trajectory.setRobotTrajectoryMsg(*move_group_interface.getCurrentState(), trajectory_msg);

            // Apply time parameterization to the trajectory
            trajectory_processing::IterativeParabolicTimeParameterization time_param;
            bool success = time_param.computeTimeStamps(robot_trajectory, max_velocity_scaling, max_acceleration_scaling);
            if (!success) {
                ROS_WARN("Failed to apply time parameterization.");
                continue;
            }

            // Convert back to moveit_msgs::RobotTrajectory
            robot_trajectory.getRobotTrajectoryMsg(trajectory_msg);

            // Verify that the time is strictly increasing between waypoints
            if (!isTrajectoryTimeIncreasing(trajectory_msg)) {
                ROS_ERROR("Trajectory time is not strictly increasing. Aborting execution.");
                continue;
            }

            // Create a new plan with the parameterized trajectory
            moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
            cartesian_plan.trajectory_ = trajectory_msg;

            // Execute the Cartesian trajectory synchronously
            moveit::planning_interface::MoveItErrorCode result = move_group_interface.execute(cartesian_plan);
            
            if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
                ROS_INFO("Successfully executed Cartesian trajectory %d.", i + 1);
            } else {
                ROS_WARN("Failed to execute Cartesian trajectory %d.", i + 1);
            }
        } else {
            ROS_WARN("Failed to compute Cartesian path for trajectory %d, skipping...", i + 1);
        }
    }
}

// Helper function to ensure time is strictly increasing
bool RobotController::isTrajectoryTimeIncreasing(moveit_msgs::RobotTrajectory &trajectory) {
    // Minimum time difference to ensure strict increasing time
    const double min_time_increment = 1e-4;  // 0.1 ms minimum increment between waypoints

    for (size_t i = 1; i < trajectory.joint_trajectory.points.size(); ++i) {
        // Check if current time is less than or equal to the previous time
        if (trajectory.joint_trajectory.points[i].time_from_start <= trajectory.joint_trajectory.points[i - 1].time_from_start) {
            // If time is not strictly increasing, adjust it by a small increment
            trajectory.joint_trajectory.points[i].time_from_start = trajectory.joint_trajectory.points[i - 1].time_from_start + ros::Duration(min_time_increment);
        }
    }
    return true;
}



//methods for thread start/stop handling
void RobotController::startJerkTrajectory(int num_moves, double max_velocity_scaling, double max_acceleration_scaling, double offScale_x, double offScale_y, double offScale_z) {
    if (jerk_running.load()) {
        ROS_WARN("Jerk trajectory is already running.");
        return;
    }

    jerk_running.store(true);

    // Start the jerk trajectory in a new thread
    jerk_thread = std::thread(&RobotController::executeJerkTrajectory, this, num_moves, max_velocity_scaling, max_acceleration_scaling, offScale_x, offScale_y, offScale_z);
}

void RobotController::stopJerkTrajectory() {
    if (!jerk_running.load()) {
        ROS_WARN("No jerk trajectory is currently running.");
        return;
    }

    // Set the flag to false to stop the jerk trajectory
    jerk_running.store(false);

    // Join the thread to ensure it has stopped
    if (jerk_thread.joinable()) {
        jerk_thread.join();
    }

    ROS_INFO("Jerk trajectory stopped.");
}

void RobotController::startRandomMove(int num_moves, int max_valid_attempts, double max_velocity_scaling, double max_acceleration_scaling, int planning_attempts, double planning_time) {
    if (random_move_running.load()) {
        ROS_WARN("Random move is already running.");
        return;
    }

    random_move_running.store(true);

    // Start the random move in a new thread
    random_move_thread = std::thread(&RobotController::moveRandom, this, num_moves, max_valid_attempts, max_velocity_scaling, max_acceleration_scaling, planning_attempts, planning_time);
}

void RobotController::stopRandomMove() {
    if (!random_move_running.load()) {
        ROS_WARN("No random move is currently running.");
        return;
    }

    // Set the flag to false to stop the random move
    random_move_running.store(false);

    // Join the thread to ensure it has stopped
    if (random_move_thread.joinable()) {
        random_move_thread.join();
    }

    ROS_INFO("Random move stopped.");
}

void RobotController::startCartesian(int num_moves, double max_velocity_scaling, double max_acceleration_scaling, double offScale_x, double offScale_y, double offScale_z) {
    if (cartesian_running.load()) {
        ROS_WARN("Cartesian trajectory is already running.");
        return;
    }

    cartesian_running.store(true);

    // Start the jerk trajectory in a new thread
    cartesian_thread = std::thread(&RobotController::executeCartesianJerkTrajectory, this, num_moves, max_velocity_scaling, max_acceleration_scaling, offScale_x, offScale_y, offScale_z);
}

void RobotController::stopCartesian() {
    if (!cartesian_running.load()) {
        ROS_WARN("No Cartesian trajectory is currently running.");
        return;
    }

    // Set the flag to false to stop the jerk trajectory
    cartesian_running.store(false);

    // Join the thread to ensure it has stopped
    if (cartesian_thread.joinable()) {
        cartesian_thread.join();
    }

    ROS_INFO("Cartesian trajectory stopped.");
}