#ifndef TERMINAL_H
#define TERMINAL_H

#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>
#include <string>
#include <vector>

class Terminal {
public:
    Terminal(ros::NodeHandle& N);

    // Render method to display the logs
    void renderPlot();

private:
    ros::NodeHandle nh;
    ros::Subscriber log_sub;  // Subscriber for log messages
    std::vector<std::string> log_messages;  // Buffer to store log messages

    // Callback to handle log messages
    void logCallback(const rosgraph_msgs::Log::ConstPtr& msg);
};

#endif // TERMINAL_H
