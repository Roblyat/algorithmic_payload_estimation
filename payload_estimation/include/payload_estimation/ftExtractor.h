#ifndef FT_EXTRACTOR_H
#define FT_EXTRACTOR_H

#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <fstream>  // For file I/O

class ftExtractor
{
public:
    ftExtractor(ros::NodeHandle& nh);   // Constructor
    ~ftExtractor();  // Destructor to close file properly

private:
    // Callback functions for both topics
    void wrenchCallback(const geometry_msgs::Wrench::ConstPtr& msg);
    void predictedWrenchCallback(const geometry_msgs::Wrench::ConstPtr& msg);

    // Subtract predicted wrench from wrench and store result
    void computeDifference();

    // ROS subscribers
    ros::Subscriber wrench_sub_;
    ros::Subscriber predicted_wrench_sub_;

    // ROS publisher (if you want to publish the result)
    ros::Publisher difference_pub_;

    // Messages to store the most recent values from the topics
    geometry_msgs::Wrench wrench_;
    geometry_msgs::Wrench predicted_wrench_;

    // Flags to check if data is received from both topics
    bool wrench_received_;
    bool predicted_wrench_received_;

    // File to save the error data
    std::ofstream outfile_;
};

#endif // FT_EXTRACTOR_H