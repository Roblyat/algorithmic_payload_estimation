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
            if (ImGui::BeginTabItem("Robot Control")) {

                ImGui::Text("Use buttons to move the robot.");  // Informational text

                //////////////////////////
                // Cartesian movement UI //
                //////////////////////////

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

                //////////////////////
                // Speed control UI //
                //////////////////////
                ImGui::SetCursorPos(ImVec2(50, 240));  // Moved down from Z buttons
                ImGui::Text("Speed Control:");
                ImGui::SetCursorPos(ImVec2(50, 260));  // Trackbar adjusted position
                ImGui::PushItemWidth(150);  // trackbar width
                ImGui::SliderInt("Speed (1-100)", &speed, 1, 100, "%d");
                ImGui::PopItemWidth();

                ///////////////////////////
                // Predefined Pose UI //
                ///////////////////////////

                ImGui::SetCursorPos(ImVec2(50, 310));  // Adjust position as per layout
                ImGui::Text("Select Pose:");
                ImGui::SetCursorPos(ImVec2(50, 330));
                ImGui::PushItemWidth(120);  // Compact width for drop-down
                if (ImGui::Combo("##pose_select", &current_pose_index, poses, IM_ARRAYSIZE(poses))) {
                    // Pose selected
                }
                ImGui::PopItemWidth();

                // Move Button for Pose
                ImGui::SetCursorPos(ImVec2(50, 370));
                if (ImGui::Button("Move", ImVec2(80.0f, 40.0f))) {
                    std::string selected_pose = poses[current_pose_index];  // Get the selected pose
                    robot_controller.execPreDef(selected_pose);
                }


                ///////////////////////////
                // Gripper Control UI    //
                ///////////////////////////

                // Gripper position control dropdown
                ImGui::SetCursorPos(ImVec2(50, 420));  // Adjust position as per layout
                ImGui::Text("Gripper Control:");
                ImGui::SetCursorPos(ImVec2(50, 440));  // Dropdown below the label
                ImGui::PushItemWidth(120);  // Compact width for drop-down
                if (ImGui::Combo("##gripper_select", &current_gripper_index, gripper_positions, IM_ARRAYSIZE(gripper_positions))) {
                    // Gripper position selected
                }
                ImGui::PopItemWidth();  // Reset width

                ///////////////////////////
                // Gripper Speed Control //
                ///////////////////////////

                // Gripper speed control slider
                ImGui::SetCursorPos(ImVec2(50, 480));  // Adjust position after the dropdown
                ImGui::Text("Gripper Speed:");
                ImGui::SetCursorPos(ImVec2(50, 500));  // Slider below the text
                ImGui::PushItemWidth(150);  // Set slider width
                ImGui::SliderFloat("##gripper_speed_slider", &gripper_speed, 0.1f, 1.0f, "%.2f");  // Slider for speed control
                ImGui::PopItemWidth();  // Reset width

                // Button to move the gripper with the selected speed
                ImGui::SetCursorPos(ImVec2(50, 540));  // Button position adjusted
                if (ImGui::Button("Move Gripper", ImVec2(120.0f, 40.0f))) {
                    std::string selected_gripper_position = gripper_positions[current_gripper_index];  // Get the selected gripper position
                    robot_controller.controlGripper(selected_gripper_position, gripper_speed);  // Call the method to move the gripper with speed control
                }
                /////////////////////////////

                ImGui::EndTabItem();  // End of Robot Control Tab
            }

            if (ImGui::BeginTabItem("Terminal")) {
                terminal.renderPlot();  // Placeholder for Terminal Tab
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