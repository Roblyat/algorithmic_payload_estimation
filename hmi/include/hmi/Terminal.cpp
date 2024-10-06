#include "Terminal.h"
#include <imgui.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>
#include <string>
#include <thread>

Terminal::Terminal(ros::NodeHandle& N) : nh(N), recording_running(false) {
    // Subscribe to the /rosout_agg topic for aggregated ROS logs
    // log_sub = nh.subscribe("/rosout_agg", 1000, &Terminal::logCallback, this);
}

Terminal::~Terminal() {
    stopRosbagRecording();  // Ensure that recording is stopped
}

/////////////////////
// Terminal Output //
/////////////////////
// Callback to handle log messages
void Terminal::logCallback(const rosgraph_msgs::Log::ConstPtr& msg) {
    // Format the log message with severity and text
    std::string severity;
    switch (msg->level) {
        case rosgraph_msgs::Log::DEBUG:
            severity = "[DEBUG]";
            break;
        case rosgraph_msgs::Log::INFO:
            severity = "[INFO]";
            break;
        case rosgraph_msgs::Log::WARN:
            severity = "[WARN]";
            break;
        case rosgraph_msgs::Log::ERROR:
            severity = "[ERROR]";
            break;
        case rosgraph_msgs::Log::FATAL:
            severity = "[FATAL]";
            break;
        default:
            severity = "[UNKNOWN]";
            break;
    }

    // Add formatted message to the log buffer
    std::string log_entry = severity + " " + msg->msg;
    log_messages.push_back(log_entry);

    // Limit the number of log entries to avoid overflowing memory
    if (log_messages.size() > 1000) {
        log_messages.erase(log_messages.begin());  // Remove oldest entry
    }
}

// Render method to display logs in the ImGui window
void Terminal::renderPlot() {
    ImGui::BeginChild("LogWindow", ImVec2(600, 400), true, ImGuiWindowFlags_HorizontalScrollbar);
    for (const auto& log : log_messages) {
        ImGui::TextUnformatted(log.c_str());
    }
    ImGui::EndChild();
}

////////////////////////
// Rosbag Recording   //
////////////////////////
// Worker method to run the rosbag recording in a separate thread
void Terminal::rosbagRecordingWorker(const std::string& full_save_path) {
    std::lock_guard<std::mutex> lock(terminal_mutex);  // Lock terminal resources

    // Construct the rosbag recording command
    std::string command = "rosbag record -O " + full_save_path + " /joint_states /wrench &";
    system(command.c_str());  // Execute the command

    while (recording_running.load()) {
        // Keep the thread alive while recording is active
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Stop the recording process when the flag is set to false
    system("pkill -f 'rosbag record'");  // Stop the rosbag recording
}

// Start rosbag recording
void Terminal::startRosbagRecording(const std::string& full_save_path) {
    if (recording_running.load()) {
        ROS_WARN("Rosbag recording is already running.");
        return;
    }

    recording_running.store(true);  // Set the flag to true

    // Start the recording in a separate thread
    recording_thread = std::thread(&Terminal::rosbagRecordingWorker, this, full_save_path);
}

// Stop rosbag recording
void Terminal::stopRosbagRecording() {
    if (!recording_running.load()) {
        ROS_WARN("No rosbag recording is currently running.");
        return;
    }

    recording_running.store(false);  // Set the flag to false to stop the recording

    // Join the thread to ensure it has stopped
    if (recording_thread.joinable()) {
        recording_thread.join();
    }

    ROS_WARN("Rosbag recording stopped.");
}

////////////////////////
// Training Node Code //
////////////////////////

// Worker method to run the Python node in a separate thread
void Terminal::pythonNodeWorker(const std::string& script_path) {
    std::lock_guard<std::mutex> lock(terminal_mutex);  // Lock terminal resources

    // Command to start the Python node
    std::string command = "python3 " + script_path + " &";  // Use python3 for running the node
    system(command.c_str());  // Convert std::string to const char* using c_str()

    while (training_running.load()) {
        // Keep the thread alive while training is running
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Stop the Python node process when the flag is set to false
    std::string stop_command = "pkill -f '" + script_path + "'";  // Construct the stop command
    system(stop_command.c_str());  // Convert std::string to const char* using c_str()
}

// Start the Python node in a separate thread
void Terminal::startPythonNode(const std::string& script_path) {
    if (training_running.load()) {
        ROS_WARN("Training node is already running.");
        return;
    }

    training_running.store(true);  // Set the flag to true

    // Start the Python node in a separate thread
    training_thread = std::thread(&Terminal::pythonNodeWorker, this, script_path);
}

// Stop the Python node
void Terminal::stopPythonNode() {
    if (!training_running.load()) {
        ROS_WARN("No training node is currently running.");
        return;
    }

    training_running.store(false);  // Set the flag to false to stop the node

    // Join the thread to ensure it has stopped
    if (training_thread.joinable()) {
        training_thread.join();
    }

    ROS_INFO("Training node stopped.");
}

// Set ROS parameters on the parameter server
void Terminal::setRosParam(const std::string& param_name, const std::string& param_value) {
    std::string command = "rosparam set " + param_name + " " + param_value;
    system(command.c_str());  // Execute the command to set the parameter
    ROS_INFO_STREAM("Set ROS parameter: " << param_name << " = " << param_value);
}