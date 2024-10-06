#ifndef TERMINAL_H
#define TERMINAL_H

#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>
#include <string>
#include <vector>
#include <cstdlib>  // for system terminal cmd
#include <mutex>    // For std::mutex
#include <thread>   // For std::thread
#include <atomic>   // For std::atomic<bool>

class Terminal {
public:
    Terminal(ros::NodeHandle& N);
    ~Terminal();  // Destructor to ensure thread cleanup

    // Render method to display the logs
    void renderPlot();

    // Start and stop rosbag recording
    void startRosbagRecording(const std::string& full_save_path);
    void stopRosbagRecording();  // Stop the rosbag recording

    //training thread
    void setRosParam(const std::string& param_name, const std::string& param_value);
    void startPythonNode(const std::string& script_path);
    void stopPythonNode();

    std::mutex terminal_mutex;  // Mutex to protect terminal operations

private:
    ros::NodeHandle nh;
    ros::Subscriber log_sub;  // Subscriber for log messages
    std::vector<std::string> log_messages;  // Buffer to store log messages

    std::atomic<bool> recording_running;  // Flag to indicate if recording is running
    std::thread recording_thread;  // Thread for rosbag recording

    std::thread training_thread;  // Thread for rosbag recording
    std::atomic<bool> training_running;  // Flag to indicate if recording is running

    // Callback to handle log messages
    void logCallback(const rosgraph_msgs::Log::ConstPtr& msg);

    // Worker method for recording
    void rosbagRecordingWorker(const std::string& full_save_path);

    // Worker method for the training node.py in a separate thread
    void pythonNodeWorker(const std::string& script_path);
};

#endif // TERMINAL_H