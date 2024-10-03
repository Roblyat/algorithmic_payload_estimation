#ifndef TERMINAL_H
#define TERMINAL_H

#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>
#include <string>
#include <vector>
#include <cstdlib>  // for system terminal cmd
#include <mutex>    // For std::mutex

class Terminal {
public:
    Terminal(ros::NodeHandle& N);

    // Render method to display the logs
    void renderPlot();

    // Start recording rosbag
    void startRosbagRecording(const std::string& full_save_path);

    std::mutex terminal_mutex;  // Mutex to protect terminal operations

private:
    ros::NodeHandle nh;
    ros::Subscriber log_sub;  // Subscriber for log messages
    std::vector<std::string> log_messages;  // Buffer to store log messages

    // Callback to handle log messages
    void logCallback(const rosgraph_msgs::Log::ConstPtr& msg);
};

#endif // TERMINAL_H
