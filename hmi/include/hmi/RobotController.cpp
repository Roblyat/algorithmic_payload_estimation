#include "RobotController.h"
#include <cstdlib>

// Constructor
RobotController::RobotController(ros::NodeHandle& N, const std::string& planning_group)
    : nh(N), move_group_interface(planning_group), current_planning_group_(planning_group),
      current_action(ActionType::EXEC_CARTESIAN), action_ready(false) {
    worker_running.store(true);
    worker_thread = std::thread(&RobotController::workerMethod, this);  // Start worker in constructor
}

// Destructor to safely stop the thread
RobotController::~RobotController()
{
    worker_running.store(false);  // Signal the worker thread to stop
    if (worker_thread.joinable()) worker_thread.join();
}

// Update action type and set ready flag
void RobotController::updateAction(ActionType action, const ActionParams& params) {
    current_action = action;
    current_params = params;  // Update parameters
    action_ready.store(true);  // Signal the worker thread to process the new action

    // Log the parameters for debugging
    ROS_INFO("Updating action to: %d", static_cast<int>(action));

    // Print ActionParams details
    ROS_INFO("Action Parameters:");
    ROS_INFO("  Pose: %s", params.pose.c_str());
    ROS_INFO("  Gripper Position: %s", params.gripper_position.c_str());
    ROS_INFO("  Number of Moves: %d", params.num_moves);
    ROS_INFO("  Max Velocity Scaling: %f", params.max_velocity_scaling);
    ROS_INFO("  Max Acceleration Scaling: %f", params.max_acceleration_scaling);
    ROS_INFO("  Offset Scale X: %f", params.offScale_x);
    ROS_INFO("  Offset Scale Y: %f", params.offScale_y);
    ROS_INFO("  Offset Scale Z: %f", params.offScale_z);
    ROS_INFO("  Sampling: %s", params.sampling ? "true" : "false");
    ROS_INFO("  Sample Goal: %s", params.sample_goal ? "true" : "false");
    ROS_INFO("  Cartesian: %s", params.cartesian ? "true" : "false");
    ROS_INFO("  Set Orientation: %s", params.setOrientation ? "true" : "false");
    ROS_INFO("  DX: %f", params.dx);
    ROS_INFO("  DY: %f", params.dy);
    ROS_INFO("  DZ: %f", params.dz);
    ROS_INFO("  Speed: %d", params.speed);
    ROS_INFO("  Move Plane: %s", params.move_plane ? "true" : "false");
    ROS_INFO("  XY Plane: %s", params.xy_plane ? "true" : "false");
    ROS_INFO("  XZ Plane: %s", params.xz_plane ? "true" : "false");
    ROS_INFO("  YZ Plane: %s", params.yz_plane ? "true" : "false");
}


// workerMethod to run continuously
void RobotController::workerMethod() {
    while (worker_running) {
        if (action_ready.load()) {
            action_ready.store(false);  // Reset the action flag
            ROS_INFO("Processing action: %d", current_action.load());

            bool action_completed = false;

            // Execute based on the current action and check completion status
            switch (current_action) {
                case EXEC_CARTESIAN:
                    action_completed = execCartesian();
                    break;

                case EXEC_RANDOM:
                    action_completed = execRandom();
                    break;

                case EXEC_PREDEF:
                    action_completed = execPreDef();
                    break;

                case MOVE_CARTESIAN:
                    action_completed = moveCartesian();
                    break;

                case MOVE_GRIPPER:
                    action_completed = controlGripper();
                    break;

                case STOP:
                    // Immediate stop, breaking out of motion handling.
                    ROS_INFO("Stopping current action.");
                    worker_running = false;  // Use this flag to exit loop or halt actions
                    action_completed = true;
                    break;

                default:
                    ROS_WARN("Unknown action specified in workerMethod.");
                    break;
            }

            if (action_completed) {
                ROS_INFO("Action %d completed successfully.", current_action.load());
            } else {
                ROS_WARN("Action %d did not complete successfully.", current_action.load());
            }
        }

        // Small sleep to avoid busy waiting
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}


// Method to dynamically set the planning group
void RobotController::setPlanningGroup(const std::string& group_name) {
    // Update the current planning group
    current_planning_group_ = group_name;

    // Reinitialize the MoveGroupInterface with the new group
    move_group_interface = moveit::planning_interface::MoveGroupInterface(group_name);
}

// Determine velocity and acceleration scaling factors based on flags
std::pair<double, double> RobotController::setScalingValues() {
    // Define scaling limits
    double standard_velocity_limit = 0.2;
    double standard_acceleration_limit = 0.3;
    double cartesian_velocity_limit = 0.3;
    double cartesian_acceleration_limit = 0.5;

    // Choose the appropriate limits based on whether it’s Cartesian or standard movement
    double velocity_limit = current_params.cartesian ? cartesian_velocity_limit : standard_velocity_limit;
    double acceleration_limit = current_params.cartesian ? cartesian_acceleration_limit : standard_acceleration_limit;

    double velocity_scaling = std::min(current_params.max_velocity_scaling, velocity_limit);
    double acceleration_scaling = std::min(current_params.max_acceleration_scaling, acceleration_limit);

    if (current_params.sampling) {
        // Expand the sampling range, e.g., from 0.05 to the effective (capped) limit
        velocity_scaling = 0.01 + (static_cast<int>((static_cast<double>(rand()) / RAND_MAX) * ((velocity_scaling - 0.01) / 0.005))) * 0.005;
        acceleration_scaling = 0.01 + (static_cast<int>((static_cast<double>(rand()) / RAND_MAX) * ((acceleration_scaling - 0.01) / 0.005))) * 0.005;;
        ROS_INFO("Using sampled velocity scaling: %.3f, sampled acceleration scaling: %.3f", velocity_scaling, acceleration_scaling);
    } else {
        ROS_INFO("Using provided velocity scaling: %.3f, provided acceleration scaling: %.3f", velocity_scaling, acceleration_scaling);
    }

    return std::make_pair(velocity_scaling, acceleration_scaling);
}

// Set the goal target with an enforced minimum displacement if moving along a single axis
void RobotController::setGoalTarget(geometry_msgs::Pose& target_pose) {
    // Count the number of non-zero offsets
    int non_zero_count = (current_params.offScale_x != 0) + (current_params.offScale_y != 0) + (current_params.offScale_z != 0);

    // If only one axis is non-zero, enforce minimum displacement
    bool enforce_min = (non_zero_count == 1);

    // Lambda function to generate random movement with optional minimum enforcement
    auto generate_random_delta = [enforce_min](double scale) -> double {
        double delta;
        do {
            delta = (static_cast<double>(rand()) / RAND_MAX - 0.5) * scale;
        } while (enforce_min && fabs(delta) < 0.2); // Enforce minimum 0.2m displacement if needed
        return delta;
    };

    // Apply the random deltas to each axis as needed
    if (current_params.offScale_x != 0) {
        target_pose.position.x += generate_random_delta(current_params.offScale_x);
    }
    if (current_params.offScale_y != 0) {
        target_pose.position.y += generate_random_delta(current_params.offScale_y);
    }
    if (current_params.offScale_z != 0) {
        target_pose.position.z += generate_random_delta(current_params.offScale_z);
    }
}


// Set a valid random orientation for the target_pose
void RobotController::setRandomOrientation(geometry_msgs::Pose& target_pose) {
    // Apply small random perturbations to the current orientation to avoid large rotations
    double perturbation_limit = 0.5;  // Define a small perturbation range

    target_pose.orientation.x += (static_cast<double>(rand()) / RAND_MAX - 0.5) * 2 * perturbation_limit;
    target_pose.orientation.y += (static_cast<double>(rand()) / RAND_MAX - 0.5) * 2 * perturbation_limit;
    target_pose.orientation.z += (static_cast<double>(rand()) / RAND_MAX - 0.5) * 2 * perturbation_limit;
    target_pose.orientation.w += (static_cast<double>(rand()) / RAND_MAX - 0.5) * 2 * perturbation_limit;

    // Normalize to ensure valid orientation
    double magnitude = sqrt(
        target_pose.orientation.x * target_pose.orientation.x +
        target_pose.orientation.y * target_pose.orientation.y +
        target_pose.orientation.z * target_pose.orientation.z +
        target_pose.orientation.w * target_pose.orientation.w
    );

    target_pose.orientation.x /= magnitude;
    target_pose.orientation.y /= magnitude;
    target_pose.orientation.z /= magnitude;
    target_pose.orientation.w /= magnitude;

    ROS_INFO("Setting a constrained random orientation.");
}


void RobotController::moveInPlane(geometry_msgs::Pose& target_pose) {
        
    // Ensure only one plane is active, with default to XY if multiple or none are set
    if (current_params.xy_plane && !current_params.xz_plane && !current_params.yz_plane) {
        current_params.offScale_z = 0;  // Zero out Z for XY plane

        ROS_WARN("OffScale values: x = %f, y = %f, z = %f", 
         current_params.offScale_x, 
         current_params.offScale_y, 
         current_params.offScale_z);

        setGoalTarget(target_pose);
        ROS_INFO("Target set within the XY plane.");

    } else if (!current_params.xy_plane && current_params.xz_plane && !current_params.yz_plane) {
        current_params.offScale_y = 0;  // Zero out Y for XZ plane

        ROS_WARN("OffScale values: x = %f, y = %f, z = %f", 
         current_params.offScale_x, 
         current_params.offScale_y, 
         current_params.offScale_z);

        setGoalTarget(target_pose);
        ROS_INFO("Target set within the XZ plane.");

    } else if (!current_params.xy_plane && !current_params.xz_plane && current_params.yz_plane) {
        current_params.offScale_x = 0;  // Zero out X for YZ plane

        ROS_WARN("OffScale values: x = %f, y = %f, z = %f", 
         current_params.offScale_x, 
         current_params.offScale_y, 
         current_params.offScale_z);

        setGoalTarget(target_pose);
        ROS_INFO("Target set within the YZ plane.");

    } else {
        // Default or invalid/multiple planes selected, default to XY plane
        ROS_WARN("Invalid or multiple planes specified. Defaulting to XY plane.");

        ROS_WARN("OffScale values: x = %f, y = %f, z = %f", 
         current_params.offScale_x, 
         current_params.offScale_y, 
         current_params.offScale_z);

        current_params.offScale_z = 0;  // Zero out Z for XY plane
        setGoalTarget(target_pose);
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

// Compute a Cartesian path with the target waypoints
bool RobotController::computeCartesianPath(std::vector<geometry_msgs::Pose>& waypoints, moveit_msgs::RobotTrajectory& trajectory_msg) {

    const double eef_step = 0.01; // Step size in meters

    // Compute the Cartesian path
    double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, trajectory_msg);

    if (fraction < 0.95) {
        ROS_WARN("Unable to plan the entire Cartesian path. Only %.2f%% of the path was planned.", fraction * 100.0);
        return false;
    }
    ROS_INFO("Successfully computed Cartesian path.");

    // Retrieve scaling values based on sampling and cartesian flags
    double velocity_scaling, acceleration_scaling;
    std::tie(velocity_scaling, acceleration_scaling) = setScalingValues();

    // Convert to robot_trajectory for time parameterization
    robot_trajectory::RobotTrajectory robot_trajectory(move_group_interface.getRobotModel(), "manipulator");
    robot_trajectory.setRobotTrajectoryMsg(*move_group_interface.getCurrentState(), trajectory_msg);

    // Apply time parameterization with the scaled values
    trajectory_processing::IterativeParabolicTimeParameterization time_param;
    if (!time_param.computeTimeStamps(robot_trajectory, velocity_scaling, acceleration_scaling)) {
        ROS_WARN("Failed to apply time parameterization.");
        return false;
    }

    // Update the trajectory with the computed scaling
    robot_trajectory.getRobotTrajectoryMsg(trajectory_msg);

    // Verify that the time is strictly increasing between waypoints
    if (!isTrajectoryTimeIncreasing(trajectory_msg)) {
        ROS_ERROR("Trajectory time is not strictly increasing. Aborting execution.");
        return false;
    }

    return true;
}

// Plan and execute the given trajectory
void RobotController::planExecute(const moveit_msgs::RobotTrajectory& trajectory_msg, int trajectory_number) {
    moveit::planning_interface::MoveGroupInterface::Plan trajectory_plan;
    trajectory_plan.trajectory_ = trajectory_msg;

    auto result = move_group_interface.execute(trajectory_plan);
    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
        ROS_INFO("Trajectory %d executed successfully.", trajectory_number);
    } else {
        ROS_WARN("Failed to execute trajectory %d.", trajectory_number);
    }
}

// Method for Cartesian movement with gui buttons
bool RobotController::moveCartesian() {
    setPlanningGroup("manipulator");

    geometry_msgs::PoseStamped current_pose = move_group_interface.getCurrentPose();
    geometry_msgs::Pose target = current_pose.pose;
    
    // Adjust target position based on the input deltas (dx, dy, dz)
    target.position.x += current_params.dx * current_params.speed * 0.001;
    target.position.y += current_params.dy * current_params.speed * 0.001;
    target.position.z += current_params.dz * current_params.speed * 0.001;

    // Create a list of waypoints (just the target in this case)
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(target);

    // Define variables for planning
    moveit_msgs::RobotTrajectory trajectory;
    const double eef_step = 0.01;  // Step size

    // Compute the Cartesian path (no jump threshold required)
    double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, trajectory);

    // Check if the path computation was successful
    if (fraction < 0.95) {
        ROS_WARN("Unable to plan the entire Cartesian path. Only %.2f%% of the path was planned.", fraction * 100.0);
        return true;
    }

    // Create a plan object to hold the trajectory
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    // Execute the planned trajectory
    moveit::core::MoveItErrorCode result = move_group_interface.execute(plan);

    // Provide feedback based on the execution result
    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
        ROS_INFO("Cartesian path executed successfully.");
    } else {
        ROS_WARN("Failed to execute the Cartesian path.");
    }
    return true;
}

// trargets are moveit predefined poses, see srdf
bool RobotController::execPreDef() {

    setPlanningGroup("manipulator");

    // Set maximum velocity and acceleration scaling
    move_group_interface.setMaxVelocityScalingFactor(0.04); 
    move_group_interface.setMaxAccelerationScalingFactor(0.015);

    // Set the target pose to the predefined position for the robot
    move_group_interface.setNamedTarget(current_params.pose);

    // Configure collision checking and set parameters for planning
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
        moveit::core::MoveItErrorCode result = move_group_interface.execute(plan);
        if (result == moveit::core::MoveItErrorCode::SUCCESS) {
            ROS_INFO("Robot is moving to the %s position asynchronously.", current_params.pose.c_str());
        } else {
            ROS_WARN("Failed to move to the %s position.", current_params.pose.c_str());
        }
    } else {
        ROS_WARN("Planning to %s failed due to collision or other constraints.", current_params.pose.c_str());
    }
    return true;
}


// Refactored executeCartesianJerkTrajectory method
bool RobotController::execCartesian() {

    setPlanningGroup("manipulator");

    for (int i = 0; i < current_params.num_moves; ++i) {

        geometry_msgs::PoseStamped current_pose = move_group_interface.getCurrentPose("sensor_robotiq_ft_frame_id");
        geometry_msgs::Pose target_pose = current_pose.pose;

        // Choose movement type based on the move_plane flag
        if (current_params.move_plane) {
            moveInPlane(target_pose);
        } else {
            setGoalTarget(target_pose);
            if (current_params.setOrientation) {
                setRandomOrientation(target_pose);
            }
        }

        // Plan a Cartesian path
        std::vector<geometry_msgs::Pose> waypoints = {current_pose.pose, target_pose};
        moveit_msgs::RobotTrajectory trajectory_msg;

        if (computeCartesianPath(waypoints, trajectory_msg)) {
            planExecute(trajectory_msg, i + 1);
        } else {
            ROS_WARN("Failed to compute Cartesian path for trajectory %d, skipping...", i + 1);
        }
    }
    return true;
}


// moveRandom function
bool RobotController::execRandom() {
    // std::lock_guard<std::mutex> lock(move_group_mutex);
    setPlanningGroup("manipulator");

    for (int i = 0; i < current_params.num_moves; ++i) {
        // if (!random_move_running.load()) {
        //     ROS_INFO("Random move stopped early.");
        //     return;
        // }

        move_group_interface.setStartStateToCurrentState();
        geometry_msgs::Pose target_pose;

        if (current_params.sample_goal) {
            // Set target within specified offsets if sample_goal is true
            geometry_msgs::PoseStamped current_pose = move_group_interface.getCurrentPose("sensor_robotiq_ft_frame_id");
            target_pose = current_pose.pose;

            if (current_params.move_plane) {
                // Use the same plane selection logic as in execCartesian
                moveInPlane(target_pose);

            } else {
                // Standard target setting within offsets
                setGoalTarget(target_pose);
                ROS_INFO("Setting sampled target within specified offsets without specific plane constraints.");
            }
        } else {
            // Generate a fully random valid pose
            target_pose = move_group_interface.getRandomPose("sensor_robotiq_ft_frame_id").pose;
            ROS_INFO("Setting fully random target within the reachable workspace.");
        }

        // Set the target pose as the goal
        move_group_interface.setPoseTarget(target_pose);

        // Set scaling values based on provided parameters
        double velocity_scaling, acceleration_scaling;

        if(current_params.cartesian) {
            current_params.cartesian = false;  // Disable Cartesian flag for random poses, used in setScalingValues
        }

        std::tie(velocity_scaling, acceleration_scaling) = setScalingValues();

        move_group_interface.setMaxVelocityScalingFactor(velocity_scaling);
        move_group_interface.setMaxAccelerationScalingFactor(acceleration_scaling);

        // Plan the motion
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            ROS_INFO("Successfully planned random pose %d with velocity scaling %.2f and acceleration scaling %.2f", i + 1, velocity_scaling, acceleration_scaling);
            planExecute(plan.trajectory_, i + 1);
            break;
        } else {
            ROS_WARN("Failed to plan random pose %d, retrying...", i + 1);
        }
    }
    return true;
}


bool RobotController::controlGripper() {
    setPlanningGroup("gripper");

    // Set the target position for the gripper (either "open" or "closed")
    move_group_interface.setNamedTarget(current_params.gripper_position);

    // Set gripper speed
    move_group_interface.setMaxVelocityScalingFactor(current_params.speed * 0.01);  // Adjust the speed (value between 0 and 1)

    // Plan and execute the movement
    moveit::core::MoveItErrorCode result = move_group_interface.move();

    // Provide feedback based on the execution result
    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
        ROS_INFO("Gripper moved to the %s position at speed %d.", current_params.gripper_position.c_str(), current_params.speed);
    } else {
        ROS_WARN("Failed to move the gripper to the %s position.", current_params.gripper_position.c_str());
    }

    return true;
}