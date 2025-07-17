#include "ftExtractor.h"
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <fstream>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

ftExtractor::ftExtractor(ros::NodeHandle& nh)
    : wrench_sub_(nh, "/wrench", 1000), predicted_wrench_sub_(nh, "/predicted_wrench", 1000), sync_(wrench_sub_, predicted_wrench_sub_, 1000)
{
    // Register synchronized callback
    sync_.registerCallback(boost::bind(&ftExtractor::synchronizedCallback, this, _1, _2));

    // Publisher for difference topic
    difference_pub_ = nh.advertise<geometry_msgs::WrenchStamped>("/wrench_difference", 1);

    // Get the rosbag name parameter from the server
    std::string rosbag_name = ros::param::param<std::string>("/rosparam/rosbag_name", "recorded_data.bag");
    std::string rosbag_base_name = rosbag_name.substr(0, rosbag_name.find_last_of('.'));

    // Open the file for logging errors
    std::string path = "/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/raw/csv/test/" + rosbag_base_name + "_error.csv";
    outfile_.open(path, std::ios::out);  // Opening file in write mode
    if (outfile_.is_open()) {
        outfile_ << "force_x_error,force_y_error,force_z_error,torque_x_error,torque_y_error,torque_z_error\n";  // CSV Header
    } else {
        ROS_ERROR("Unable to open file for writing");
    }
}

ftExtractor::~ftExtractor() {
    if (outfile_.is_open()) {
        outfile_.close();  // Close the file stream in the destructor
    }
}

void ftExtractor::synchronizedCallback(const geometry_msgs::WrenchStamped::ConstPtr& wrench_msg,
                                       const geometry_msgs::WrenchStamped::ConstPtr& predicted_wrench_msg)
{
    wrench_ = *wrench_msg;
    predicted_wrench_ = *predicted_wrench_msg;

    // Debug: Print received wrench messages
    // ROS_INFO_STREAM("Received /wrench message:\n" << *wrench_msg);
    // ROS_INFO_STREAM("Received /predicted_wrench message:\n" << *predicted_wrench_msg);

    computeDifference();
}

void ftExtractor::computeDifference()
{
    try {
        // Calculate the difference between wrench and predicted wrench
        geometry_msgs::WrenchStamped difference_msg;
        difference_msg.header.stamp = ros::Time::now();
        difference_msg.header.frame_id = wrench_.header.frame_id;  // Optional: use the same frame

        difference_msg.wrench.force.x = wrench_.wrench.force.x - predicted_wrench_.wrench.force.x;
        difference_msg.wrench.force.y = wrench_.wrench.force.y - predicted_wrench_.wrench.force.y;
        difference_msg.wrench.force.z = wrench_.wrench.force.z - predicted_wrench_.wrench.force.z;
        difference_msg.wrench.torque.x = wrench_.wrench.torque.x - predicted_wrench_.wrench.torque.x;
        difference_msg.wrench.torque.y = wrench_.wrench.torque.y - predicted_wrench_.wrench.torque.y;
        difference_msg.wrench.torque.z = wrench_.wrench.torque.z - predicted_wrench_.wrench.torque.z;

        // Publish the difference
        difference_pub_.publish(difference_msg);

        // Log the error data to a file
        if (outfile_.is_open()) {
            outfile_ << difference_msg.wrench.force.x << "," << difference_msg.wrench.force.y << "," << difference_msg.wrench.force.z << ","
                     << difference_msg.wrench.torque.x << "," << difference_msg.wrench.torque.y << "," << difference_msg.wrench.torque.z << "\n";
        } else {
            ROS_ERROR("Unable to open file for writing");
        }

    } catch (const ros::serialization::StreamOverrunException& e) {
        ROS_ERROR("Stream Overrun Exception in computeDifference: %s", e.what());
    }
}