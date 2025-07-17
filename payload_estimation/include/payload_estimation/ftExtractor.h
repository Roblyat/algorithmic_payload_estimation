#ifndef FT_EXTRACTOR_H
#define FT_EXTRACTOR_H

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <fstream>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

class ftExtractor
{
public:
    ftExtractor(ros::NodeHandle& nh);   // Constructor
    ~ftExtractor();  // Destructor to close file properly

private:
    // Synchronized callback for both topics
    void synchronizedCallback(const geometry_msgs::WrenchStamped::ConstPtr& wrench_msg,
                              const geometry_msgs::WrenchStamped::ConstPtr& predicted_wrench_msg);

    // Subtract predicted wrench from wrench and store result
    void computeDifference();

    // ROS publisher
    ros::Publisher difference_pub_;

    // File to save the error data
    std::ofstream outfile_;

    // Messages to store the most recent values from the topics
    geometry_msgs::WrenchStamped wrench_;
    geometry_msgs::WrenchStamped predicted_wrench_;

    // Message filter subscribers and synchronizer
    message_filters::Subscriber<geometry_msgs::WrenchStamped> wrench_sub_;
    message_filters::Subscriber<geometry_msgs::WrenchStamped> predicted_wrench_sub_;
    message_filters::TimeSynchronizer<geometry_msgs::WrenchStamped, geometry_msgs::WrenchStamped> sync_;
};

#endif // FT_EXTRACTOR_H