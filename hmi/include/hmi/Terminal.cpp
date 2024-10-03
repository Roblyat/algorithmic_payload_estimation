#include "Terminal.h"
#include <imgui.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>
#include <string>

Terminal::Terminal(ros::NodeHandle& N) : nh(N) {
    // Subscribe to the /rosout_agg topic for aggregated ROS logs
    log_sub = nh.subscribe("/rosout_agg", 1000, &Terminal::logCallback, this);
}

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

void Terminal::startRosbagRecording(const std::string& full_save_path) {
    std::lock_guard<std::mutex> lock(terminal_mutex);  // Locking terminal resources
    std::string command = "rosbag record -O " + full_save_path + " /joint_states /wrench &";
    system(command.c_str());  // Execute the command
}
