#include "Control.h"

// Constructor
Control::Control(ros::NodeHandle& N)
    : nh(N), button_30x10(300, 100), button_5x5(50, 50)
{
    // Initialize subscribers and publishers
    // poseSub = nh.subscribe("measurement_points", 1, &Control::poseCallback, this);
}

///////////////////////
// Render Function   //
// Button definition //
///////////////////////

void Control::render()
{

    // Create a text input box for the planning group
    ImGui::InputText("Planning Group", planning_group, IM_ARRAYSIZE(planning_group));

    ImGui::SetCursorPos(ImVec2(450, 50)); // X, Y
    if (ImGui::Button("Init Pos", button_5x5))
    {
        ROS_WARN_STREAM("Moving " << planning_group << " to init position");

        // Check if the planning group is not empty and not "manipulator"
        if (std::string(planning_group).empty() || std::string(planning_group) == "manipulator")
        {
            // Move the robot to init position and open the gripper
            moveInit(std::string(planning_group));
            controlGripper("open");  // Open the gripper
        }
        else
        {
            ROS_WARN("Invalid planning group provided, skipping motion.");
        }

        return;
    }

    ImGui::SetCursorPos(ImVec2(150, 0)); // X, Y
    if (ImGui::Button("Y+", button_5x5))
    {
        moveLinYPos();
    }

    ImGui::SetCursorPos(ImVec2(150, 300)); // X, Y
    if (ImGui::Button("Y-", button_5x5))
    {
        moveLinYNeg();
    }

    ImGui::SetCursorPos(ImVec2(0, 150)); // X, Y
    if (ImGui::Button("X-", button_5x5))
    {
        moveLinXNeg();
    }

    ImGui::SetCursorPos(ImVec2(300, 150)); // X, Y
    if (ImGui::Button("X+", button_5x5))
    {
        moveLinXPos();
    }

    ImGui::SetCursorPos(ImVec2(150, 100)); // X, Y
    if (ImGui::Button("Z+", button_5x5))
    {
        moveLinZPos();
    }

    ImGui::SetCursorPos(ImVec2(150, 200)); // X, Y
    if (ImGui::Button("Z-", button_5x5))
    {
        moveLinZNeg();
    }

    // Add Gripper Control Buttons
    ImGui::SetCursorPos(ImVec2(450, 300)); // X, Y for "Gripper Open"
    if (ImGui::Button("Gripper Open", button_5x5))
    {
        ROS_WARN("Opening gripper");
        controlGripper("open");  // Call controlGripper with "open"
    }

    ImGui::SetCursorPos(ImVec2(450, 350)); // X, Y for "Gripper Close"
    if (ImGui::Button("Gripper Close", button_5x5))
    {
        ROS_WARN("Closing gripper");
        controlGripper("close");  // Call controlGripper with "close"
    }
}


void Control::moveInit(const std::string& planning_group)
{
    // Initialize MoveGroupInterface with the specified planning group
    moveit::planning_interface::MoveGroupInterface move_group(planning_group);

    // Set the reference frame for planning
    move_group.setPoseReferenceFrame("base_link");

    // Create a target pose object
    geometry_msgs::Pose target_pose;

    // Set translation values (position)
    target_pose.position.x = 0.816;  // x from tf_echo for ur5_base_link to sensor_robotiq_ft_frame_id
    target_pose.position.y = 0.229;  // y from tf_echo
    target_pose.position.z = -0.014; // z from tf_echo

    // Set rotation values (orientation) in Quaternion format
    target_pose.orientation.x = -0.504; // x from tf_echo quaternion
    target_pose.orientation.y = -0.496; // y from tf_echo quaternion
    target_pose.orientation.z = -0.496; // z from tf_echo quaternion
    target_pose.orientation.w = 0.504;  // w from tf_echo quaternion

    // Set the target pose
    move_group.setPoseTarget(target_pose);

    // Plan and execute the motion
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success)
    {
        move_group.move();
        ROS_INFO("Robot moved to the target pose.");
    }
    else
    {
        ROS_WARN("Planning to the target pose failed.");
    }
}


// Linear movement functions
void Control::moveLinXPos()
{   
    std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    geometry_msgs::PoseStamped cp = move_group.getCurrentPose();
    waypoints.clear();
    geometry_msgs::Pose target = cp.pose;
    target.position.x += 0.001 * speed;
    waypoints.push_back(target);

    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    move_group.asyncExecute(trajectory);
}

void Control::moveLinXNeg()
{
    std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    geometry_msgs::PoseStamped cp = move_group.getCurrentPose();
    waypoints.clear();
    geometry_msgs::Pose target = cp.pose;
    target.position.x -= 0.001 * speed;
    waypoints.push_back(target);

    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    move_group.asyncExecute(trajectory);
}

void Control::moveLinYPos()
{
    std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    geometry_msgs::PoseStamped cp = move_group.getCurrentPose();
    waypoints.clear();
    geometry_msgs::Pose target = cp.pose;
    target.position.y += 0.001 * speed;
    waypoints.push_back(target);

    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    move_group.asyncExecute(trajectory);
}

void Control::moveLinYNeg()
{
    std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    geometry_msgs::PoseStamped cp = move_group.getCurrentPose();
    waypoints.clear();
    geometry_msgs::Pose target = cp.pose;
    target.position.y -= 0.001 * speed;
    waypoints.push_back(target);

    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    move_group.asyncExecute(trajectory);
}

void Control::moveLinZPos()
{
    std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    geometry_msgs::PoseStamped cp = move_group.getCurrentPose();
    waypoints.clear();
    geometry_msgs::Pose target = cp.pose;
    target.position.z += 0.001 * speed;
    waypoints.push_back(target);

    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    move_group.asyncExecute(trajectory);
}

void Control::moveLinZNeg()
{
    std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    geometry_msgs::PoseStamped cp = move_group.getCurrentPose();
    waypoints.clear();
    geometry_msgs::Pose target = cp.pose;
    target.position.z -= 0.001 * speed;
    waypoints.push_back(target);

    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    move_group.asyncExecute(trajectory);
}

void Control::moveRobotToPose(const std::string& planning_group, const geometry_msgs::Pose& target_pose)
{
    moveit::planning_interface::MoveGroupInterface move_group(planning_group);

    double planning_time = 5.0;
    nh.param("planning_time", planning_time, planning_time);
    move_group.setPlanningTime(planning_time);
    ROS_INFO("Planning time set to: %f seconds", planning_time);

    int planning_attempts = 5;
    nh.param("planning_attempts", planning_attempts, planning_attempts);
    move_group.setNumPlanningAttempts(planning_attempts);
    ROS_INFO("Planning attempts set to: %d", planning_attempts);

    // move_group.setPoseReferenceFrame("base_link");

    // Set the maximum velocity and acceleration scaling factors
    move_group.setMaxVelocityScalingFactor(velocity_scaling_factor);
    move_group.setMaxAccelerationScalingFactor(acceleration_scaling_factor);

    move_group.setPoseTarget(target_pose);

    ROS_WARN("Moving robot to target pose... %f %f %f", target_pose.position.x, target_pose.position.y, target_pose.position.z);

    // Plan to the target pose
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
    if (success)
    {
        // Execute the planned trajectory
        move_group.move();
    }
    else
    {
        ROS_WARN("Planning failed!");
    }

    move_group.clearPoseTargets();
}

geometry_msgs::Pose Control::handlePose(int index, const geometry_msgs::PoseArray& poses)
{
    // Simply return the pose at the given index
    return poses.poses[index];
}

void Control::controlGripper(const std::string& action)
{
    // Initialize the MoveGroupInterface for the gripper
    moveit::planning_interface::MoveGroupInterface gripper_group("gripper");

    // Check whether to open or close the gripper
    if (action == "open")
    {
        ROS_INFO("Opening the gripper...");
        gripper_group.setNamedTarget("open");  // Use predefined "open" position from MoveIt Setup Assistant
    }
    else if (action == "close")
    {
        ROS_INFO("Closing the gripper...");
        gripper_group.setNamedTarget("close"); // Use predefined "close" position from MoveIt Setup Assistant
    }
    else
    {
        ROS_WARN("Invalid action. Use 'open' or 'close'.");
        return;
    }

    // Plan and execute the motion
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (gripper_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success)
    {
        gripper_group.move();
        ROS_INFO("Gripper moved to the %s position.", action.c_str());
    }
    else
    {
        ROS_WARN("Failed to move the gripper to the %s position.", action.c_str());
    }
}
