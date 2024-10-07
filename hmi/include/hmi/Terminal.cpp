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

//////////////////////////
// Rosbag to CSV Worker //
//////////////////////////
void Terminal::rosbagToCSVWorker() {
    std::lock_guard<std::mutex> lock(terminal_mutex);  // Lock terminal resources

    // Command to process rosbag to CSV
    std::string command = "rosrun payload_estimation rosbag_to_csv_split.py &";  // Replace with the actual command or script
    system(command.c_str());  // Execute the command

    while (rosbag_to_csv_running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Ensure process stops when flag is false
    system("pkill -f 'rosbag_to_csv'");
}

void Terminal::startRosbagToCSV() {
    if (rosbag_to_csv_running.load()) {
        ROS_WARN("Rosbag to CSV process is already running.");
        return;
    }

    rosbag_to_csv_running.store(true);  // Set flag to true
    rosbag_to_csv_thread = std::thread(&Terminal::rosbagToCSVWorker, this);
}

void Terminal::stopRosbagToCSV() {
    if (!rosbag_to_csv_running.load()) {
        ROS_WARN("No rosbag to CSV process is currently running.");
        return;
    }

    rosbag_to_csv_running.store(false);  // Set flag to false

    if (rosbag_to_csv_thread.joinable()) {
        rosbag_to_csv_thread.join();
    }

    ROS_INFO("Rosbag to CSV process stopped.");
}

//////////////////////////////
// CSV Preprocessing Worker //
//////////////////////////////
void Terminal::csvPreprocessingWorker() {
    std::lock_guard<std::mutex> lock(terminal_mutex);  // Lock terminal resources

    // Command to run CSV preprocessing
    std::string command = "rosrun payload_estimation csv_preprocess_data.py &";  // Replace with actual script path
    system(command.c_str());  // Execute the command

    while (csv_preprocessing_running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    system("pkill -f 'preprocessing_script.py'");
}

void Terminal::startCSVPreprocessing() {
    if (csv_preprocessing_running.load()) {
        ROS_WARN("CSV preprocessing is already running.");
        return;
    }

    csv_preprocessing_running.store(true);  // Set flag to true
    csv_preprocessing_thread = std::thread(&Terminal::csvPreprocessingWorker, this);
}

void Terminal::stopCSVPreprocessing() {
    if (!csv_preprocessing_running.load()) {
        ROS_WARN("No CSV preprocessing is currently running.");
        return;
    }

    csv_preprocessing_running.store(false);  // Set flag to false

    if (csv_preprocessing_thread.joinable()) {
        csv_preprocessing_thread.join();
    }

    ROS_INFO("CSV preprocessing stopped.");
}


////////////////////////
// Training Node Code //
////////////////////////

void Terminal::gpTrainingWorker() {
    std::lock_guard<std::mutex> lock(terminal_mutex);  // Lock terminal resources

    // Command to start the Python node using rosrun
    std::string command = "rosrun payload_estimation gp_training_node.py &";  // Replace with actual package and script name
    system(command.c_str());  // Execute the command

    while (training_running.load()) {
        // Keep the thread alive while training is running
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Stop the Python node process when the flag is set to false
    std::string stop_command = "pkill -f 'gp_training_node.py'";  // Construct the stop command for the specific node
    system(stop_command.c_str());  // Execute the stop command
}


// Start the Python node in a separate thread
void Terminal::startTraining() {
    if (training_running.load()) {
        ROS_WARN("Training node is already running.");
        return;
    }

    training_running.store(true);  // Set the flag to true

    // Start the Python node in a separate thread
    training_thread = std::thread(&Terminal::gpTrainingWorker, this);
}

// Stop the Python node
void Terminal::stopTraining() {
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