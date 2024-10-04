#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <stdio.h>
#define GL_SILENCE_DEPRECATION
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif
#include <GLFW/glfw3.h> 
#include <iostream>
#include <string>
#include <regex>

// ROS and MoveIt
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// Add RobotController and Plotting header files
#include "RobotController.h"
#include "Terminal.h"

//Thread
#include <thread>

#if defined(_MSC_VER) && (_MSC_VER >= 1900) && !defined(IMGUI_DISABLE_WIN32_FUNCTIONS)
#pragma comment(lib, "legacy_stdio_definitions")
#endif

// Error callback for GLFW
static void glfw_error_callback(int error, const char* description) {
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

// Main code
int main(int argc, char** argv) {

    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
        return 1;

    // Decide GL+GLSL versions
#if defined(IMGUI_IMPL_OPENGL_ES2)
    // GL ES 2.0 + GLSL 100
    const char* glsl_version = "#version 100";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
#elif defined(__APPLE__)
    // GL 3.2 + GLSL 150
    const char* glsl_version = "#version 150";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // Required on Mac
#else
    // GL 3.0 + GLSL 130
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    //glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
    //glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ only
#endif

    // Create window with graphics context
    GLFWwindow* window = glfwCreateWindow(700, 700, "Example HMI for Robotermodellierung", NULL, NULL);
    if (window == NULL)
        return 1;
    glfwSetWindowSizeLimits(window, 700, 700, 700, 700);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsLight();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);
    ImVec4 clear_color = ImVec4(0.40f, 0.50f, 0.60f, 0.45f);
    
    
    // Initialize ROS and create node handle
    ros::init(argc, argv, "hmi");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    // Create instances of RobotController and Plotting classes
    RobotController robot_controller(nh, "manipulator");
    Terminal terminal(nh);
    int speed = 1;

    // List of predefined poses
    const char* poses[] = { "Init", "home", "up" };
    static int current_pose_index = 0;
    // List of gripper positions
    const char* gripper_positions[] = { "open", "closed" };
    static int current_gripper_index = 0;
    static float gripper_speed = 0.5;  // Default speed (between 0 and 1)

    //randomMove method parameters
    static int max_random_valid_attempts = 5;  // Default value
    static double moveit_planning_time = 5.0;  // Default value
    static int moveit_planning_attempts = 10;  // Default value
    //randomMove and executeJerkTrajectory parameters
    static int random_moves_amount = 5;  // Default value
    static double max_velocity_scaling = 0.4;  // Default value
    static double max_acceleration_scaling = 0.4;  // Default value
    //executeJerkTrajectory parameters
    static double offScale_x = 0.1;
    static double offScale_y = 0.1;
    static double offScale_z = 0.1;
    // Define static strings to hold the save path and file name
    static char save_path[256] = "/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data";       // Default directory path
    static char file_name[128] = "recorded_data.bag";  // Default file name

    //move buttons as block
    int y_bRand = 200;

    // Main loop
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        // Start new frame for ImGui
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Create a window with tabs
        ImGui::SetNextWindowSize(ImVec2(600, 600));
        ImGui::SetNextWindowPos(ImVec2(50, 50));
        ImGui::Begin("Robot HMI", NULL, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

        if (ImGui::BeginTabBar("Tabs")) {

            //////////////////////////////////////////////////////////////////////////////////
            //////////////////////////// Control Tab ////////////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////
            if (ImGui::BeginTabItem("Robot Control")) {

                ImGui::Text("Use buttons to move the robot.");  // Informational text

                ///////////////////////////
                // Cartesian movement UI //
                ///////////////////////////
                // X-axis buttons (X+ and X-)
                ImGui::SetCursorPosX(50);
                if (ImGui::Button("X+", ImVec2(50.0f, 50.0f))) robot_controller.moveCartesian(1, 0, 0, speed);
                ImGui::SameLine();
                ImGui::SetCursorPosX(120);
                if (ImGui::Button("X-", ImVec2(50.0f, 50.0f))) robot_controller.moveCartesian(-1, 0, 0, speed);

                // Y-axis buttons (Y+ and Y-)
                ImGui::SetCursorPosX(50);
                if (ImGui::Button("Y+", ImVec2(50.0f, 50.0f))) robot_controller.moveCartesian(0, 1, 0, speed);
                ImGui::SameLine();
                ImGui::SetCursorPosX(120);
                if (ImGui::Button("Y-", ImVec2(50.0f, 50.0f))) robot_controller.moveCartesian(0, -1, 0, speed);

                // Z-axis buttons (Z+ and Z-)
                ImGui::SetCursorPosX(50);
                if (ImGui::Button("Z+", ImVec2(50.0f, 50.0f))) robot_controller.moveCartesian(0, 0, 1, speed);
                ImGui::SameLine();
                ImGui::SetCursorPosX(120);
                if (ImGui::Button("Z-", ImVec2(50.0f, 50.0f))) robot_controller.moveCartesian(0, 0, -1, speed);

                ////////////////////////////////
                // Cartesian Speed control UI //
                ////////////////////////////////
                ImGui::SetCursorPos(ImVec2(50, 240));  // Moved down from Z buttons
                ImGui::Text("Speed Control:");
                ImGui::SetCursorPos(ImVec2(50, 260));  // Trackbar adjusted position
                ImGui::PushItemWidth(150);  // trackbar width
                ImGui::SliderInt("Speed (1-100)", &speed, 1, 100, "%d");
                ImGui::PopItemWidth();

                ///////////////////
                // Predefined UI //
                ///////////////////
                ImGui::SetCursorPos(ImVec2(50, 310));  // Adjust position as per layout
                ImGui::Text("Select Pose:");
                ImGui::SetCursorPos(ImVec2(50, 330));
                ImGui::PushItemWidth(120);  // Compact width for drop-down
                if (ImGui::Combo("##pose_select", &current_pose_index, poses, IM_ARRAYSIZE(poses))) {
                    // Pose selected
                }
                ImGui::PopItemWidth();

                // Move Button for Predefined Pose
                ImGui::SetCursorPos(ImVec2(50, 370));
                if (ImGui::Button("Move PreDef", ImVec2(100.0f, 40.0f))) {  // Increased width of button for better appearance
                    std::string selected_pose = poses[current_pose_index];  // Get the selected pose
                    robot_controller.execPreDef(selected_pose);
                }
                
                ////////////////////
                // Random Pose UI //
                ////////////////////
                ImGui::SetCursorPos(ImVec2(50, 420));  // Spacing adjusted for consistency
                ImGui::Text("Random Pose Execution Settings");

                // Input box for number of random moves (moves to execute)
                ImGui::SetCursorPos(ImVec2(50, 440));
                ImGui::PushItemWidth(100);  // Set width for the input box
                ImGui::InputInt("Moves to Execute", &random_moves_amount);
                ImGui::PopItemWidth();  // Reset width

                // Input box for max planning attempts (how often to retry planning)
                ImGui::SetCursorPos(ImVec2(50, 470));
                ImGui::PushItemWidth(100);  // Set width for the input box
                ImGui::InputInt("Max RandValid Attempts", &max_random_valid_attempts);
                ImGui::PopItemWidth();  // Reset width

                // Ensure the input value for planning attempts is at least 1

                // Input box for max velocity scaling
                ImGui::SetCursorPos(ImVec2(350, 530 - y_bRand));
                ImGui::PushItemWidth(100);  // Set width for the input box
                ImGui::InputDouble("Max Vel Scale", &max_velocity_scaling, 0.01, 0.1, "%.2f");
                ImGui::PopItemWidth();  // Reset width

                // Input box for max acceleration scaling
                ImGui::SetCursorPos(ImVec2(350, 560 - y_bRand));
                ImGui::PushItemWidth(100);  // Set width for the input box
                ImGui::InputDouble("Max Acc Scale", &max_acceleration_scaling, 0.01, 0.1, "%.2f");
                ImGui::PopItemWidth();  // Reset width

                // Input box for max planning_time
                ImGui::SetCursorPos(ImVec2(350, 590 - y_bRand));
                ImGui::PushItemWidth(100);  // Set width for the input box
                ImGui::InputDouble("Planning Time (s)", &moveit_planning_time, 0.1, 0.5, "%.1f");
                ImGui::PopItemWidth();  // Reset width
                // Ensure the input value for planning time is at least 1

                // Input box for number of planning attempts per move
                ImGui::SetCursorPos(ImVec2(350, 620 - y_bRand));
                ImGui::PushItemWidth(100);  // Set width for the input box
                ImGui::InputInt("Planning Attempts", &moveit_planning_attempts);
                ImGui::PopItemWidth();  // Reset width

                // Input box for offScale_x
                ImGui::SetCursorPos(ImVec2(350, 650 - y_bRand));
                ImGui::PushItemWidth(100);  // Set width for the input box
                ImGui::InputDouble("offScale_x", &offScale_x);
                ImGui::PopItemWidth();  // Reset width

                // Input box for offScale_y
                ImGui::SetCursorPos(ImVec2(350, 680 - y_bRand));
                ImGui::PushItemWidth(100);  // Set width for the input box
                ImGui::InputDouble("offScale_y", &offScale_y);
                ImGui::PopItemWidth();  // Reset width

                // Input box for offScale_y
                ImGui::SetCursorPos(ImVec2(350, 710 - y_bRand));
                ImGui::PushItemWidth(100);  // Set width for the input box
                ImGui::InputDouble("offScale_z", &offScale_z);
                ImGui::PopItemWidth();  // Reset width

                // Button to execute the random moves
                ImGui::SetCursorPos(ImVec2(350, 300 - y_bRand));  // Moved to the right side of the Random Pose settings
                if (ImGui::Button("Move Random", ImVec2(120.0f, 40.0f))) {
                    robot_controller.startRandomMove(random_moves_amount, max_random_valid_attempts, max_velocity_scaling, max_acceleration_scaling, 
                                                        moveit_planning_attempts, moveit_planning_time);
                }

                // Button to stop random move
                ImGui::SetCursorPos(ImVec2(350, 360 - y_bRand));
                if (ImGui::Button("Stop Random", ImVec2(120.0f, 40.0f))) {
                    robot_controller.stopRandomMove();
                }


                // Button to execute the jerk trajectory
                ImGui::SetCursorPos(ImVec2(350, 420 - y_bRand));  // Moved to the right side of the Random Pose settings
                if (ImGui::Button("Move Jerk", ImVec2(120.0f, 40.0f))) {
                    robot_controller.startJerkTrajectory(random_moves_amount, max_velocity_scaling, max_acceleration_scaling, offScale_x, offScale_y, offScale_z);
                }

                // Button to stop jerk trajectory
                ImGui::SetCursorPos(ImVec2(350, 480 - y_bRand));
                if (ImGui::Button("Stop Jerk", ImVec2(120.0f, 40.0f))) {
                    robot_controller.stopJerkTrajectory();
                }
                                

                ///////////////////////////
                // Gripper Control UI    //
                ///////////////////////////
                // Gripper position control dropdown
                ImGui::SetCursorPos(ImVec2(50, 510));  // Adjust position as per layout
                ImGui::Text("Gripper Control:");
                ImGui::SetCursorPos(ImVec2(50, 530));  // Dropdown below the label
                ImGui::PushItemWidth(120);  // Compact width for drop-down
                if (ImGui::Combo("##gripper_select", &current_gripper_index, gripper_positions, IM_ARRAYSIZE(gripper_positions))) {
                    // Gripper position selected
                }
                ImGui::PopItemWidth();  // Reset width

                ///////////////////////////
                // Gripper Speed Control //
                ///////////////////////////
                // Gripper speed control slider
                ImGui::SetCursorPos(ImVec2(50, 570));  // Adjust position after the dropdown
                ImGui::Text("Gripper Speed:");
                ImGui::SetCursorPos(ImVec2(50, 590));  // Slider below the text
                ImGui::PushItemWidth(150);  // Set slider width
                ImGui::SliderFloat("##gripper_speed_slider", &gripper_speed, 0.1f, 1.0f, "%.2f");  // Slider for speed control
                ImGui::PopItemWidth();  // Reset width

                // Button to move the gripper with the selected speed
                ImGui::SetCursorPos(ImVec2(50, 620));  // Button position adjusted
                if (ImGui::Button("Move Gripper", ImVec2(120.0f, 40.0f))) {
                    std::string selected_gripper_position = gripper_positions[current_gripper_index];  // Get the selected gripper position
                    robot_controller.controlGripper(selected_gripper_position, gripper_speed);  // Call the method to move the gripper with speed control
                }
                ///////////////////////////

                ImGui::EndTabItem();  // End of Robot Control Tab
            }


            //////////////////////////////////////////////////////////////////////////////////
            //////////////////////////// Terminal Tab ////////////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////
            if (ImGui::BeginTabItem("Terminal")) {
                
                terminal.renderPlot();  // Placeholder for Terminal Tab
                
                /////////////////////////
                // Rosbag recording UI //
                /////////////////////////
                ImGui::SetCursorPos(ImVec2(350, 500));
                ImGui::Text("Save Path:");
                ImGui::InputText("##SavePath", save_path, IM_ARRAYSIZE(save_path));  // Text input for the save path

                ImGui::SetCursorPos(ImVec2(350, 525));
                ImGui::Text("File Name:");
                ImGui::InputText("##FileName", file_name, IM_ARRAYSIZE(file_name));  // Text input for the file name

                // Button to start rosbag recording
                ImGui::SetCursorPos(ImVec2(450, 500));
                if (ImGui::Button("Start Recording", ImVec2(120.0f, 40.0f))) {
                    // Pass the full path (directory + file name) to the recording function
                    std::string full_save_path = std::string(save_path) + "/" + std::string(file_name);
                    terminal.startRosbagRecording(full_save_path);
                }

                // Button to stop rosbag recording
                ImGui::SetCursorPos(ImVec2(450, 550));
                if (ImGui::Button("Stop Recording", ImVec2(120.0f, 40.0f))) {
                    terminal.stopRosbagRecording();  // Stop the recording
                }

                ImGui::EndTabItem();
            }

            ImGui::EndTabBar();  // End of Tab Bar
        }

        ImGui::End();

        // Rendering
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    // Cleanup
    ros::shutdown();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}