#include <GLFW/glfw3.h>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <ros/ros.h>
#include "Frontend.h"

#if defined(_MSC_VER) && (_MSC_VER >= 1900) && !defined(IMGUI_DISABLE_WIN32_FUNCTIONS)
#pragma comment(lib, "legacy_stdio_definitions")
#endif
   

static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

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


    //ROS Stuff
    ros::init(argc, argv, "hmi");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    static const std::string PLANNING_GROUP = "manipulator"; //#####################################################################
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP); 
    moveit_msgs::RobotTrajectory trajectory;
    std::vector<geometry_msgs::Pose> waypoints;         //#####################################################################
    int speed = 1;


    // Main loop
    while (!glfwWindowShouldClose(window))

    {
        glfwPollEvents();

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();


        // 2. Show a simple window that we create ourselves. We use a Begin/End pair to create a named window.
        
			ImGui::SetNextWindowSize(ImVec2(600,600));
			ImGui::SetNextWindowPos(ImVec2(50,50));
			
			//Start of the Window
            ImGui::Begin("Example HMI" , NULL, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse | 4 | 256 | 1);
            // Buttons return true when clicked (most widgets return true when edited/activated)
            
            

            
            //Close Button
            ImGui::SetCursorPos(ImVec2(500,500));
            if(ImGui::Button("CLOSE"))
            {
                    glfwWindowShouldClose(window);
                    break;
            }
            
            //X+ Button
            ImGui::SetCursorPos(ImVec2(275,50));
			if(ImGui::Button("X+", ImVec2(50.0,50.0)))
			{
			   geometry_msgs::PoseStamped cp = move_group_interface.getCurrentPose();
               waypoints.clear();
               geometry_msgs::Pose target = cp.pose;
               target.position.x += 0.001 * speed;
               waypoints.push_back(target);
               moveit_msgs::RobotTrajectory trajectory;
               const double jump_threshold = 0.0;
               const double eef_step = 0.01;
               double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
               move_group_interface.asyncExecute(trajectory);          
			
			}
			
			//Y- Button
			ImGui::SetCursorPos(ImVec2(200,125)); 
         	if(ImGui::Button("Y-", ImVec2(50.0,50.0)))
         	{
         	   geometry_msgs::PoseStamped cp = move_group_interface.getCurrentPose();
               waypoints.clear();
               geometry_msgs::Pose target = cp.pose;
               target.position.y -= 0.001 * speed;
               waypoints.push_back(target);
               moveit_msgs::RobotTrajectory trajectory;
               const double jump_threshold = 0.0;
               const double eef_step = 0.01;
               double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
               move_group_interface.asyncExecute(trajectory);   
         	}
         	                    
            
            //Y+ Button
            ImGui::SameLine();
            ImGui::SetCursorPos(ImVec2(350,125)); 
            if(ImGui::Button("Y+", ImVec2(50.0,50.0)))
            {
               geometry_msgs::PoseStamped cp = move_group_interface.getCurrentPose();
               waypoints.clear();
               geometry_msgs::Pose target = cp.pose;
               target.position.y += 0.001 * speed;
               waypoints.push_back(target);
               moveit_msgs::RobotTrajectory trajectory;
               const double jump_threshold = 0.0;
               const double eef_step = 0.01;
               double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
               move_group_interface.asyncExecute(trajectory); 
            }
            
            //X- Button
            ImGui::SetCursorPos(ImVec2(275,200));
            if(ImGui::Button("X-", ImVec2(50.0,50.0)))
            {
               geometry_msgs::PoseStamped cp = move_group_interface.getCurrentPose();
               waypoints.clear();
               geometry_msgs::Pose target = cp.pose;
               target.position.x -= 0.001 * speed;
               waypoints.push_back(target);
               moveit_msgs::RobotTrajectory trajectory;
               const double jump_threshold = 0.0;
               const double eef_step = 0.01;
               double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
               move_group_interface.asyncExecute(trajectory);  
            }
            
            //Z+ Button
            ImGui::SetCursorPos(ImVec2(425,75));
            if(ImGui::Button("Z+", ImVec2(50,50)))
            {
               geometry_msgs::PoseStamped cp = move_group_interface.getCurrentPose();
               waypoints.clear();
               geometry_msgs::Pose target = cp.pose;
               target.position.z += 0.001 * speed;
               waypoints.push_back(target);
               moveit_msgs::RobotTrajectory trajectory;
               const double jump_threshold = 0.0;
               const double eef_step = 0.01;
               double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
               move_group_interface.asyncExecute(trajectory);  
            }
            
            //Z- Button
            ImGui::SetCursorPos(ImVec2(425,175));
            if(ImGui::Button("Z-", ImVec2(50,50)))
            {
               geometry_msgs::PoseStamped cp = move_group_interface.getCurrentPose();
               waypoints.clear();
               geometry_msgs::Pose target = cp.pose;
               target.position.z -= 0.001 * speed;
               waypoints.push_back(target);
               moveit_msgs::RobotTrajectory trajectory;
               const double jump_threshold = 0.0;
               const double eef_step = 0.01;
               double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
               move_group_interface.asyncExecute(trajectory);  
            }
            
            
            //Stepsize Slider
            ImGui::SetCursorPos(ImVec2(50,350));
            ImGui::SliderInt("Steppsize(%)",&speed, 1,100);

            //Init Button
            ImGui::SetCursorPos(ImVec2(50,450));
            if(ImGui::Button("Init", ImVec2(50,50)))
            {
                // Set the target pose to the home position for the specified robot
                move_group_interface.setNamedTarget("home");

                // Execute the motion asynchronously (non-blocking)
                moveit::planning_interface::MoveItErrorCode result = move_group_interface.asyncMove();  // Non-blocking move

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
            
            
            // For Grippers 
            /*
            ImGui::SetCursorPos(ImVec2(50,75));
            ImGui::Button("Gripper Open", ImVec2(100,50));
            ImGui::SetCursorPos(ImVec2(50,175));
            ImGui::Button("Gripper Close", ImVec2(100,50));
            */

			//Framerate for HMI Window 
			ImGui::SetCursorPos(ImVec2(5,5));
            ImGui::Text("%.2f",ImGui::GetIO().Framerate);
            
             
            ImGui::End();
            //End of the Window



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