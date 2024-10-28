#include "Terminal.h"
#include <imgui.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>
#include <string>
#include <thread>

Terminal::Terminal(ros::NodeHandle& N) : nh(N), train_recording_running(false), test_recording_running(false) {
    // Subscribe to the /rosout_agg topic for aggregated ROS logs
    // log_sub = nh.subscribe("/rosout_agg", 1000, &Terminal::logCallback, this);
}

Terminal::~Terminal() {
    stopTrainRosbagRecording();  // Ensure that recording is stopped
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

//////////////////////////
// ROS Parameter Setter //
//////////////////////////
// Set ROS parameters on the parameter server
void Terminal::setRosParam(const std::string& param_name, const std::string& param_value) {
    std::string command = "rosparam set " + param_name + " " + param_value;
    system(command.c_str());  // Execute the command to set the parameter
    ROS_INFO_STREAM("Set ROS parameter: " << param_name << " = " << param_value);
}

////////////////////////////
// Rosbag Train Recording //
////////////////////////////
// Worker method to run the rosbag recording in a separate thread
void Terminal::trainRosbagRecordingWorker(const std::string& full_save_path) {
    std::lock_guard<std::mutex> lock(terminal_mutex);  // Lock terminal resources

    // Construct the rosbag recording command
    std::string command = "rosbag record -O " + full_save_path + " /joint_states /wrench &";
    system(command.c_str());  // Execute the command

    while (train_recording_running.load()) {
        // Keep the thread alive while recording is active
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Stop the recording process when the flag is set to false
    system("pkill -f 'rosbag record'");  // Stop the rosbag recording
}

// Start rosbag recording
void Terminal::startTrainRosbagRecording(const std::string& full_save_path) {
    if (train_recording_running.load()) {
        ROS_WARN("Rosbag train recording is already running.");
        return;
    }

    train_recording_running.store(true);  // Set the flag to true

    // Start the recording in a separate thread
    train_recording_thread = std::thread(&Terminal::trainRosbagRecordingWorker, this, full_save_path);
}

// Stop rosbag recording
void Terminal::stopTrainRosbagRecording() {
    if (!train_recording_running.load()) {
        ROS_WARN("No train rosbag recording is currently running.");
        return;
    }

    train_recording_running.store(false);  // Set the flag to false to stop the recording

    // Join the thread to ensure it has stopped
    if (train_recording_thread.joinable()) {
        train_recording_thread.join();
    }

    ROS_WARN("Rosbag train recording stopped.");
}

////////////////////////////
// Rosbag Test Recording //
////////////////////////////
// Worker method to run the rosbag recording in a separate thread
void Terminal::testRosbagRecordingWorker(const std::string& full_save_path) {
    std::lock_guard<std::mutex> lock(terminal_mutex);  // Lock terminal resources

    // Construct the rosbag recording command
    std::string command = "rosbag record -O " + full_save_path + " /predicted_effort /predicted_wrench /wrench /joint_states &";
    system(command.c_str());  // Execute the command

    while (test_recording_running.load()) {
        // Keep the thread alive while recording is active
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Stop the recording process when the flag is set to false
    system("pkill -f 'rosbag record'");  // Stop the rosbag recording
}

// Start rosbag recording
void Terminal::startTestRosbagRecording(const std::string& full_save_path) {
    if (test_recording_running.load()) {
        ROS_WARN("Rosbag test recording is already running.");
        return;
    }

    test_recording_running.store(true);  // Set the flag to true

    // Start the recording in a separate thread
    test_recording_thread = std::thread(&Terminal::testRosbagRecordingWorker, this, full_save_path);
}

// Stop rosbag recording
void Terminal::stopTestRosbagRecording() {
    if (!test_recording_running.load()) {
        ROS_WARN("No test rosbag recording is currently running.");
        return;
    }

    test_recording_running.store(false);  // Set the flag to false to stop the recording

    // Join the thread to ensure it has stopped
    if (test_recording_thread.joinable()) {
        test_recording_thread.join();
    }

    ROS_WARN("Rosbag test recording stopped.");
}

//////////////////////////
// Rosbag to CSV Worker //
//////////////////////////
void Terminal::trainRosbagToCSVWorker() {
    std::lock_guard<std::mutex> lock(terminal_mutex);  // Lock terminal resources

    // Command to process rosbag to CSV
    std::string command = "rosrun payload_estimation train_rosbag_to_csv_split.py &";  // Replace with the actual command or script
    system(command.c_str());  // Execute the command

    while (train_rosbag_to_csv_running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Ensure process stops when flag is false
    system("pkill -f 'rosbag_to_csv'");
}

void Terminal::startTrainRosbagToCSV() {
    if (train_rosbag_to_csv_running.load()) {
        ROS_WARN("Train rosbag to CSV process is already running.");
        return;
    }

    train_rosbag_to_csv_running.store(true);  // Set flag to true
    train_rosbag_to_csv_thread = std::thread(&Terminal::trainRosbagToCSVWorker, this);
}

void Terminal::stopTrainRosbagToCSV() {
    if (!train_rosbag_to_csv_running.load()) {
        ROS_WARN("No train rosbag to CSV process is currently running.");
        return;
    }

    train_rosbag_to_csv_running.store(false);  // Set flag to false

    if (train_rosbag_to_csv_thread.joinable()) {
        train_rosbag_to_csv_thread.join();
    }

    ROS_INFO("Train rosbag to CSV process stopped.");
}

//////////////////////////
// Rosbag to CSV Worker //
//////////////////////////
void Terminal::testRosbagToCSVWorker() {
    std::lock_guard<std::mutex> lock(terminal_mutex);  // Lock terminal resources

    // Command to process rosbag to CSV
    std::string command = "rosrun payload_estimation test_rosbag_to_csv_split.py &";  // Replace with the actual command or script
    system(command.c_str());  // Execute the command

    while (test_rosbag_to_csv_running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Ensure process stops when flag is false
    system("pkill -f 'rosbag_to_csv'");
}

void Terminal::startTestRosbagToCSV() {
    if (test_rosbag_to_csv_running.load()) {
        ROS_WARN("Test rosbag to CSV process is already running.");
        return;
    }

    test_rosbag_to_csv_running.store(true);  // Set flag to true
    test_rosbag_to_csv_thread = std::thread(&Terminal::testRosbagToCSVWorker, this);
}

void Terminal::stopTestRosbagToCSV() {
    if (!test_rosbag_to_csv_running.load()) {
        ROS_WARN("No test rosbag to CSV process is currently running.");
        return;
    }

    test_rosbag_to_csv_running.store(false);  // Set flag to false

    if (test_rosbag_to_csv_thread.joinable()) {
        test_rosbag_to_csv_thread.join();
    }

    ROS_INFO("Test rosbag to CSV process stopped.");
}

/////////////////////////////////////
// CSV Wrench Preprocessing Worker //
/////////////////////////////////////
void Terminal::csvWrenchPreprocessingWorker() {
    std::lock_guard<std::mutex> lock(terminal_mutex);  // Lock terminal resources

    // Command to run CSV preprocessing
    std::string command = "rosrun payload_estimation csv_preprocess_wrench_data.py &";  // Replace with actual script path
    system(command.c_str());  // Execute the command

    while (wrench_preprocessing_running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    system("pkill -f 'wrench_preprocessing_script.py'");
}

void Terminal::startWrenchPreprocessing() {
    if (wrench_preprocessing_running.load()) {
        ROS_WARN("CSV Wrench preprocessing is already running.");
        return;
    }

    wrench_preprocessing_running.store(true);  // Set flag to true
    wrench_preprocessing_thread = std::thread(&Terminal::csvWrenchPreprocessingWorker, this);
}

void Terminal::stopWrenchPreprocessing() {
    if (!wrench_preprocessing_running.load()) {
        ROS_WARN("No CSV Wrench preprocessing is currently running.");
        return;
    }

    wrench_preprocessing_running.store(false);  // Set flag to false

    if (wrench_preprocessing_thread.joinable()) {
        wrench_preprocessing_thread.join();
    }

    ROS_INFO("CSV Wrench preprocessing stopped.");
}

/////////////////////////////////////
// CSV Effort Preprocessing Worker //
/////////////////////////////////////
void Terminal::csvEffortPreprocessingWorker() {
    std::lock_guard<std::mutex> lock(terminal_mutex);  // Lock terminal resources

    // Command to run CSV preprocessing
    std::string command = "rosrun payload_estimation csv_preprocess_effort_data.py &";  // Replace with actual script path
    system(command.c_str());  // Execute the command

    while (effort_preprocessing_running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    system("pkill -f 'effort_preprocessing_script.py'");
}

void Terminal::startEffortPreprocessing() {
    if (effort_preprocessing_running.load()) {
        ROS_WARN("CSV Wrench preprocessing is already running.");
        return;
    }

    effort_preprocessing_running.store(true);  // Set flag to true
    effort_preprocessing_thread = std::thread(&Terminal::csvEffortPreprocessingWorker, this);
}

void Terminal::stopEffortPreprocessing() {
    if (!effort_preprocessing_running.load()) {
        ROS_WARN("No CSV Effort preprocessing is currently running.");
        return;
    }

    effort_preprocessing_running.store(false);  // Set flag to false

    if (effort_preprocessing_thread.joinable()) {
        effort_preprocessing_thread.join();
    }

    ROS_INFO("CSV Effort preprocessing stopped.");
}


////////////////////////
// Training Node Code //
////////////////////////

void Terminal::gpTrainingWorker(bool use_kfold, bool use_gplvm) {
    std::lock_guard<std::mutex> lock(terminal_mutex);  // Lock terminal resources

    // Command to start the appropriate Python node using rosrun
    std::string command;
    if (use_gplvm) {
        command = "rosrun payload_estimation gplvm_kfold_training_node.py &";  // GPLVM training node
    } else if (use_kfold) {
        command = "rosrun payload_estimation gp_kfold_training_node.py &";  // KFold training node
    } else {
        command = "rosrun payload_estimation gp_training_node.py &";  // Regular training node
    }

    system(command.c_str());  // Execute the command

    while (training_running.load()) {
        // Keep the thread alive while training is running
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Stop the Python node process when the flag is set to false
    std::string stop_command = "pkill -f 'gp_training_node.py'";  // Stop regular node
    system(stop_command.c_str());

    // Ensure kfold and gplvm nodes are killed if they were running
    std::string stop_kfold_command = "pkill -f 'gp_kfold_training_node.py'";
    std::string stop_gplvm_command = "pkill -f 'gplvm_kfold_training_node.py'";

    system(stop_kfold_command.c_str());
    system(stop_gplvm_command.c_str());
}

// start the python node
void Terminal::startTraining() {
    if (training_running.load()) {
        ROS_WARN("Training node is already running.");
        return;
    }

    // Retrieve the 'use_kfold' ROS parameter
    bool use_kfold = false;
    if (ros::param::get("/rosparam/use_kfold", use_kfold)) {
        ROS_INFO("Retrieved 'use_kfold' parameter: %s", use_kfold ? "true" : "false");
    } else {
        ROS_WARN("Failed to retrieve 'use_kfold' parameter. Using default: false");
    }

    // Retrieve the 'use_gplvm' ROS parameter
    bool use_gplvm = false;
    if (ros::param::get("/rosparam/use_gplvm", use_gplvm)) {
        ROS_INFO("Retrieved 'use_gplvm' parameter: %s", use_gplvm ? "true" : "false");
    } else {
        ROS_WARN("Failed to retrieve 'use_gplvm' parameter. Using default: false");
    }

    training_running.store(true);  // Set the flag to true

    // Start the Python node in a separate thread, passing the `use_kfold` and `use_gplvm` flags
    training_thread = std::thread(&Terminal::gpTrainingWorker, this, use_kfold, use_gplvm);
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


///////////////////////
// Testing Node Code //
///////////////////////

void Terminal::gpTestingWorker() {
    std::lock_guard<std::mutex> lock(terminal_mutex);  // Lock terminal resources

    // Command to start the Python node using rosrun
    std::string command = "rosrun payload_estimation gp_test_node.py &";  // Replace with actual package and script name
    system(command.c_str());  // Execute the command

    while (testing_running.load()) {
        // Keep the thread alive while training is running
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Stop the Python node process when the flag is set to false
    std::string stop_command = "pkill -f 'gp_test_node.py'";  // Construct the stop command for the specific node
    system(stop_command.c_str());  // Execute the stop command
}


// Start the Python node in a separate thread
void Terminal::startTesting() {
    if (testing_running.load()) {
        ROS_WARN("Testing node is already running.");
        return;
    }

    testing_running.store(true);  // Set the flag to true

    // Start the Python node in a separate thread
    testing_thread = std::thread(&Terminal::gpTestingWorker, this);
}

// Stop the Python node
void Terminal::stopTesting() {
    if (!testing_running.load()) {
        ROS_WARN("No testing node is currently running.");
        return;
    }

    testing_running.store(false);  // Set the flag to false to stop the node

    // Join the thread to ensure it has stopped
    if (testing_thread.joinable()) {
        testing_thread.join();
    }

    ROS_INFO("Testing node stopped.");
}

//////////////////////////
// Plotting Effort Code //
//////////////////////////

void Terminal::gpEffortPlotWorker() {
    std::lock_guard<std::mutex> lock(terminal_mutex);  // Lock terminal resources

    // Command to start the Python node using rosrun
    std::string command = "rosrun payload_estimation gp_effort_plots_node.py &";  // Replace with actual package and script name
    system(command.c_str());  // Execute the command

    while (effort_plot_running.load()) {
        // Keep the thread alive while training is running
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Stop the Python node process when the flag is set to false
    std::string stop_command = "pkill -f 'gp_effort_plots_node.py'";  // Construct the stop command for the specific node
    system(stop_command.c_str());  // Execute the stop command

    // Ensure to close any open plots
    // system("pkill -f 'matplotlib'");  // Forcefully close matplotlib windows if they remain open
}


// Start the Python node in a separate thread
void Terminal::startEffortPlot() {
    if (effort_plot_running.load()) {
        ROS_WARN("Effort plot node is already running.");
        return;
    }

    effort_plot_running.store(true);  // Set the flag to true

    // Start the Python node in a separate thread
    effort_plot_thread = std::thread(&Terminal::gpEffortPlotWorker, this);
}

// Stop the Python node
void Terminal::stopEffortPlot() {
    if (!effort_plot_running.load()) {
        ROS_WARN("No effort plot node is currently running.");
        return;
    }

    effort_plot_running.store(false);  // Set the flag to false to stop the node

    // Join the thread to ensure it has stopped
    if (effort_plot_thread.joinable()) {
        effort_plot_thread.join();
    }

    ROS_INFO("Effort plot node stopped.");
}

//////////////////////////
// Plotting Wrench Code //
//////////////////////////

void Terminal::gpWrenchPlotWorker() {
    std::lock_guard<std::mutex> lock(terminal_mutex);  // Lock terminal resources

    // Command to start the Python node using rosrun
    std::string command = "rosrun payload_estimation gp_wrench_plots_node.py &";  // Replace with actual package and script name
    system(command.c_str());  // Execute the command

    while (wrench_plot_running.load()) {
        // Keep the thread alive while training is running
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Stop the Python node process when the flag is set to false
    std::string stop_command = "pkill -f 'gp_wrench_plots_node.py'";  // Construct the stop command for the specific node
    system(stop_command.c_str());  // Execute the stop command
}


// Start the Python node in a separate thread
void Terminal::startWrenchPlot() {
    if (wrench_plot_running.load()) {
        ROS_WARN("Wrench plot node is already running.");
        return;
    }

    wrench_plot_running.store(true);  // Set the flag to true

    // Start the Python node in a separate thread
    wrench_plot_thread = std::thread(&Terminal::gpWrenchPlotWorker, this);
}

// Stop the Python node
void Terminal::stopWrenchPlot() {
    if (!wrench_plot_running.load()) {
        ROS_WARN("No wrench plot node is currently running.");
        return;
    }

    wrench_plot_running.store(false);  // Set the flag to false to stop the node

    // Join the thread to ensure it has stopped
    if (wrench_plot_thread.joinable()) {
        wrench_plot_thread.join();
    }

    ROS_INFO("Wrench plot node stopped.");
}