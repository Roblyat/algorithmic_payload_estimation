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

enum RobotAction {
    ACTION_NONE,
    ACTION_MOVE_INIT,
    ACTION_GRIPPER_OPEN,
    ACTION_GRIPPER_CLOSE,
    ACTION_MOVE_LIN_Y_POS,
    ACTION_MOVE_LIN_Y_NEG,
    ACTION_MOVE_LIN_X_POS,
    ACTION_MOVE_LIN_X_NEG,
    ACTION_MOVE_LIN_Z_POS,
    ACTION_MOVE_LIN_Z_NEG
};

void Control::render()
{
    static bool action_pending = false;  // A flag to prevent multiple calls
    RobotAction action = ACTION_NONE;

    // Position buttons in a row (or column) and set corresponding actions
    ImGui::SetCursorPos(ImVec2(10, 10)); // X, Y
    if (ImGui::Button("Init Pos", button_5x5)) action = ACTION_MOVE_INIT;
    
    ImGui::SetCursorPos(ImVec2(100, 10)); // X, Y
    if (ImGui::Button("Y+", button_5x5)) action = ACTION_MOVE_LIN_Y_POS;

    ImGui::SetCursorPos(ImVec2(100, 100)); // X, Y
    if (ImGui::Button("Y-", button_5x5)) action = ACTION_MOVE_LIN_Y_NEG;

    ImGui::SetCursorPos(ImVec2(200, 10)); // X, Y
    if (ImGui::Button("X+", button_5x5)) action = ACTION_MOVE_LIN_X_POS;

    ImGui::SetCursorPos(ImVec2(200, 100)); // X, Y
    if (ImGui::Button("X-", button_5x5)) action = ACTION_MOVE_LIN_X_NEG;

    ImGui::SetCursorPos(ImVec2(300, 10)); // X, Y
    if (ImGui::Button("Z+", button_5x5)) action = ACTION_MOVE_LIN_Z_POS;

    ImGui::SetCursorPos(ImVec2(300, 100)); // X, Y
    if (ImGui::Button("Z-", button_5x5)) action = ACTION_MOVE_LIN_Z_NEG;

    ImGui::SetCursorPos(ImVec2(400, 10)); // X, Y
    if (ImGui::Button("Gripper Open", button_5x5)) action = ACTION_GRIPPER_OPEN;

    ImGui::SetCursorPos(ImVec2(400, 100)); // X, Y
    if (ImGui::Button("Gripper Close", button_5x5)) action = ACTION_GRIPPER_CLOSE;

    // Use switch-case to handle the selected action
    switch (action)
    {
        case ACTION_MOVE_INIT:
            ROS_WARN("Moving to init position");
            moveInit();
            controlGripper("open");
            break;

        case ACTION_MOVE_LIN_Y_POS:
            moveLinYPos();
            break;

        case ACTION_MOVE_LIN_Y_NEG:
            moveLinYNeg();
            break;

        case ACTION_MOVE_LIN_X_POS:
            moveLinXPos();
            break;

        case ACTION_MOVE_LIN_X_NEG:
            moveLinXNeg();
            break;

        case ACTION_MOVE_LIN_Z_POS:
            moveLinZPos();
            break;

        case ACTION_MOVE_LIN_Z_NEG:
            moveLinZNeg();
            break;

        case ACTION_GRIPPER_OPEN:
            ROS_WARN("Opening gripper");
            controlGripper("open");
            break;

        case ACTION_GRIPPER_CLOSE:
            ROS_WARN("Closing gripper");
            controlGripper("close");
            break;

        default:
            // No action selected
            break;
    }
}


void Control::moveInit()
{
    // Initialize Move Group Interface for the robot with the "manipulator" planning group
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");

    // Set the target pose to the home position for the specified robot
    move_group.setNamedTarget("home");

    // Execute the motion asynchronously (non-blocking)
    moveit::planning_interface::MoveItErrorCode result = move_group.asyncMove();  // Non-blocking move

    // Check if the command was successfully sent
    if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        ROS_INFO("Robot is moving to the home position asynchronously.");
    }
    else
    {
        ROS_WARN("Failed to move to the home position.");
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
