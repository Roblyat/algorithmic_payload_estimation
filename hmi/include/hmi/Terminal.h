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

    //training thread
    void setRosParam(const std::string& param_name, const std::string& param_value);

    // Start and stop rosbag recording
    void startRosbagRecording(const std::string& full_save_path);
    void stopRosbagRecording();  // Stop the rosbag recording

    // Start and stop training 
    void startTraining(); 
    void stopTraining();

    // Start and stop rosbag to CSV process
    void startCSVPreprocessing();  // Start CSV preprocessing
    void stopCSVPreprocessing();  // Stop CSV preprocessing

    // Start and stop rosbag to CSV process
    void startRosbagToCSV();  // Start rosbag to CSV process
    void stopRosbagToCSV();  // Stop rosbag to CSV process

    std::mutex terminal_mutex;  // Mutex to protect terminal operations

private:
    ros::NodeHandle nh;
    ros::Subscriber log_sub;  // Subscriber for log messages
    std::vector<std::string> log_messages;  // Buffer to store log messages

    // Callback to handle log messages
    void logCallback(const rosgraph_msgs::Log::ConstPtr& msg);

    // Worker method for recording
    std::atomic<bool> recording_running;  // Flag to indicate if recording is running
    std::thread recording_thread;  // Thread for rosbag recording
    void rosbagRecordingWorker(const std::string& full_save_path);

    // Worker method for the training node.py in a separate thread
    std::thread training_thread;  // Thread for rosbag recording
    std::atomic<bool> training_running;  // Flag to indicate if recording is running
    void gpTrainingWorker();

    // Rosbag to CSV
    std::atomic<bool> rosbag_to_csv_running;  // Flag for rosbag to CSV process
    std::thread rosbag_to_csv_thread;  // Thread for rosbag to CSV process
    void rosbagToCSVWorker();  // Worker method for rosbag to CSV process

    // CSV Preprocessing
    std::atomic<bool> csv_preprocessing_running;  // Flag for CSV preprocessing
    std::thread csv_preprocessing_thread;  // Thread for CSV preprocessing
    void csvPreprocessingWorker();  // Worker method for CSV preprocessing
};

#endif // TERMINAL_H