#include "Control.h"

Control::Control(ros::NodeHandle &N) : nh(N)
{
    indexControlURProgramPub = nh.advertise<std_msgs::Int32>("hmi/control_ur_program", 10);
    poseSub = nh.subscribe("measurement_points", 1, &Control::poseCallback, this);
}

void Control::render()
{
    // Set button size
    ImVec2 button_size(300, 100); // Width, Height

    // Set position and create Setup button
    ImGui::SetCursorPos(ImVec2(50, 50)); // X, Y
    if (ImGui::Button("Setup", button_size))
    {
        setupButtonCallback();
    }


    // Set position and create Begin Measurement button
    ImGui::SetCursorPos(ImVec2(50, 200)); // X, Y
    if (ImGui::Button("Begin External Control", button_size))
    {
        for (int i = 0; i < target_poses.poses.size(); ++i)
        {
            geometry_msgs::Pose pose = handlePose(i, target_poses);
            moveRobotToPose("ur10_manipulator", pose);
        }
    }

    // Set position and create default button
    ImGui::SetCursorPos(ImVec2(50, 350)); // X, Y
    if (ImGui::Button("Default QUANTEC", button_size))
    {
        defaultQUANTECCallback();
    }

    // Set position and create default button
    ImGui::SetCursorPos(ImVec2(50, 500)); // X, Y
    if (ImGui::Button("Default UR10", button_size))
    {
        defaultUR10Callback();
    }
}

void Control::setupButtonCallback()
{
    std_msgs::Int32 msg;
    msg.data = setupValue;
    indexControlURProgramPub.publish(msg);
    ROS_INFO("Setup button pressed, published value: %d", setupValue);
}

void Control::beginExternalControlButtonCallback()
{
    std_msgs::Int32 msg;
    msg.data = externalControlValue;
    indexControlURProgramPub.publish(msg);
    ROS_INFO("Begin Measurement button pressed, published value: %d", externalControlValue);
}

void Control::moveRobotToHome(const std::string& planning_group, const std::string& target_pose)
{
    // Initialize Move Group Interface for the robot
    moveit::planning_interface::MoveGroupInterface move_group(planning_group);

    // Set goal tolerance for the robot to prevent aborts due to small errors
    double goal_tolerance;

    nh.param("goal_tolerance", goal_tolerance, 0.1);  // Default value is 0.1 if parameter not set
    move_group.setGoalTolerance(goal_tolerance);

    // Set the target pose to the home position for the specified robot
    move_group.setNamedTarget(target_pose);

    // Start movement asynchronously
    moveit::planning_interface::MoveItErrorCode result = move_group.asyncMove();

    // Check if the command was successfully sent
    if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        ROS_INFO("%s robot is moving to home position.", planning_group.c_str());
    }
    else
    {
        ROS_ERROR("Failed to start movement for %s robot. Error code: %d", planning_group.c_str(), result.val);
    }
}


void Control::defaultUR10Callback()
{
    std_msgs::Int32 msg;
    msg.data = defaultValue;
    indexControlURProgramPub.publish(msg);
    ROS_INFO("Default Setting button pressed, published value: %d", defaultValue);

    // Then move UR10
    ROS_WARN("Moving UR10 to home...");
    moveRobotToHome("ur10_manipulator", "ur10_home");
}

void Control::defaultQUANTECCallback()
{
    std_msgs::Int32 msg;
    msg.data = defaultValue;
    indexControlURProgramPub.publish(msg);
    ROS_INFO("Default Setting button pressed, published value: %d", defaultValue);

    // Move Quantec first
    ROS_WARN("Moving Quantec to home...");
    moveRobotToHome("quantec_manipulator", "quantec_home");
}

void Control::poseCallback(const geometry_msgs::PoseArray::ConstPtr &msg)
{
    if(!msg->poses.empty()){
        target_poses = *msg;
    }
}

void Control::moveRobotToPose(const std::string& planning_group, const geometry_msgs::Pose& target_pose)
{
    std_msgs::Int32 msg;
    msg.data = externalControlValue;
    indexControlURProgramPub.publish(msg);
    ROS_WARN("Begin Measurement button pressed, published value: %d", externalControlValue);

    moveit::planning_interface::MoveGroupInterface move_group(planning_group);

    double goal_position_tolerance = 0.01; 
    nh.param("goal_position_tolerance", goal_position_tolerance, goal_position_tolerance);
    move_group.setGoalPositionTolerance(goal_position_tolerance);
    ROS_INFO("Goal position tolerance set to: %f meters", goal_position_tolerance);

    double goal_orientation_tolerance = 0.01; 
    nh.param("goal_orientation_tolerance", goal_orientation_tolerance, goal_orientation_tolerance);
    move_group.setGoalOrientationTolerance(goal_orientation_tolerance);
    ROS_INFO("Goal orientation tolerance set to: %f radians", goal_orientation_tolerance);

    double planning_time = 5.0;
    nh.param("planning_time", planning_time, planning_time);
    move_group.setPlanningTime(planning_time);
    ROS_INFO("Planning time set to: %f seconds", planning_time);

    int planning_attempts = 5;
    nh.param("planning_attempts", planning_attempts, planning_attempts);
    move_group.setNumPlanningAttempts(planning_attempts);
    ROS_INFO("Planning attempts set to: %d", planning_attempts);

    move_group.setPoseReferenceFrame("base_link");

    move_group.setPoseTarget(target_pose);

    ROS_WARN("Moving robot to target pose... %f %f %f", target_pose.position.x, target_pose.position.y, target_pose.position.z);

    move_group.asyncMove();

    move_group.clearPoseTargets();
}

geometry_msgs::Pose Control::handlePose(int index, const geometry_msgs::PoseArray& poses)
{
    // Simply return the pose at the given index
    return poses.poses[index];
}

void Control::moveLIN(const std::string& planning_group, const std::string &referenceLink) {

    moveit::planning_interface::MoveGroupInterface move_group(planning_group);

    double linearDistance;
    double acceptableFraction;
    std::string plannerId;
    int planningAttempts;
    double planningTime;
    double goalPositionTolerance;
    double goalOrientationTolerance;
    double linearVelocityScaling;
    double linearAccelerationScaling;

    // Load parameters from the parameter server (YAML file)
    nh.param("goal_position_tolerance", goalPositionTolerance, 0.01);
    nh.param("goal_orientation_tolerance", goalOrientationTolerance, 0.05);
    nh.param("planning_time", planningTime, 20.0);
    nh.param("planning_attempts", planningAttempts, 100);
    nh.param("acceptable_fraction", acceptableFraction, 0.9);
    nh.param("planner_id", plannerId, std::string("PRM"));
    nh.param("linear_distance", linearDistance, 0.1);
    nh.param("linear_velocity_scaling", linearVelocityScaling, 0.1);
    nh.param("linear_acceleration_scaling", linearAccelerationScaling, 0.1);

    // Apply velocity and acceleration scaling
    move_group.setMaxVelocityScalingFactor(linearVelocityScaling);
    move_group.setMaxAccelerationScalingFactor(linearAccelerationScaling);

    geometry_msgs::PoseStamped currentPose = move_group.getCurrentPose(referenceLink);

    // Set the reference frame to base_link
    move_group.setPoseReferenceFrame("table_link");

    // Transformation of the current pose
    tf2::Transform currentTransform;
    tf2::fromMsg(currentPose.pose, currentTransform);

    // Define translation along the Z-axis / set distance in Controle_Panel.h
    tf2::Vector3 translation(0, 0, distance);
    tf2::Vector3 translatedVector = currentTransform.getBasis() * translation;

    // Define a new target pose based on the translation
    geometry_msgs::Pose targetPose = currentPose.pose;
    targetPose.position.x += translatedVector.x();
    targetPose.position.y += translatedVector.y();
    targetPose.position.z += translatedVector.z();

    // Visualize the target pose in relation to the baseline link
    // visualToolsBase.publishAxisLabeled(targetPose, "target_pose_base");
    // visualToolsBase.trigger(); // Display changes in RViz

    // Create a vector for waypoints and add only the target pose (no initial pose)
    std::vector<geometry_msgs::Pose> waypoints;
    // waypoints.push_back(currentPose.pose);
    waypoints.push_back(targetPose); // Only the end pose

    // Compute the Cartesian path
    moveit_msgs::RobotTrajectory trajectory;
    const double jumpThreshold = 0.0; // No jumps allowed
    const double eefStep = 0.01;      // Step size for the end effector
    double fraction = move_group.computeCartesianPath(waypoints, eefStep, jumpThreshold, trajectory);

    // If sufficient path is planned
    if (fraction > acceptableFraction)
    {
        // Optional: time parameterization
        robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(), move_group.getName());
        rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);

        trajectory_processing::IterativeParabolicTimeParameterization iptp;
        bool success = iptp.computeTimeStamps(rt, linearVelocityScaling, linearAccelerationScaling);

        if (success)
        {
            rt.getRobotTrajectoryMsg(trajectory);

            // Create and execute the plan
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;

            ROS_INFO_NAMED("move_group", "Planning successful. Executing the motion asynchronously...");

            // Async movement: Non-blocking, starts executing and returns immediately
            move_group.asyncMove();

            // If you prefer blocking behavior (wait until motion finishes), use this:
            // moveGroup.move();
        }
        else
        {
            ROS_WARN_NAMED("move_group", "Time parameterization failed. Unable to move to target pose.");
        }
    }
    else
    {
        ROS_WARN_NAMED("move_group", "Planning failed. The Cartesian path was not sufficiently achieved.");
    }

    // Reset the reference frame back to base_link
    move_group.setPoseReferenceFrame("base_link");
};