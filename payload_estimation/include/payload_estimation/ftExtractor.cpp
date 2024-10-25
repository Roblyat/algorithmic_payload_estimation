#include "ft_extractor.h"
#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <fstream>

ftExtractor::ftExtractor(ros::NodeHandle& nh)
: wrench_received_(false), predicted_wrench_received_(false)
{
    // Subscribe to /wrench and /predicted_wrench topics
    wrench_sub_ = nh.subscribe("/wrench", 1, &ftExtractor::wrenchCallback, this);
    predicted_wrench_sub_ = nh.subscribe("/predicted_wrench", 1, &ftExtractor::predictedWrenchCallback, this);

    // Optionally, you can advertise a topic to publish the difference result
    difference_pub_ = nh.advertise<geometry_msgs::Wrench>("/wrench_difference", 1);

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

void ftExtractor::wrenchCallback(const geometry_msgs::Wrench::ConstPtr& msg)
{
    wrench_ = *msg;
    wrench_received_ = true;

    // Check if both messages are received before computing the difference
    if (predicted_wrench_received_)
    {
        computeDifference();
    }
}

void ftExtractor::predictedWrenchCallback(const geometry_msgs::Wrench::ConstPtr& msg)
{
    predicted_wrench_ = *msg;
    predicted_wrench_received_ = true;

    // Check if both messages are received before computing the difference
    if (wrench_received_)
    {
        computeDifference();
    }
}

void ftExtractor::computeDifference()
{
    // Subtract predicted wrench from wrench
    geometry_msgs::Wrench difference;
    difference.force.x = wrench_.force.x - predicted_wrench_.force.x;
    difference.force.y = wrench_.force.y - predicted_wrench_.force.y;
    difference.force.z = wrench_.force.z - predicted_wrench_.force.z;
    difference.torque.x = wrench_.torque.x - predicted_wrench_.torque.x;
    difference.torque.y = wrench_.torque.y - predicted_wrench_.torque.y;
    difference.torque.z = wrench_.torque.z - predicted_wrench_.torque.z;

    // Publish the difference (optional)
    difference_pub_.publish(difference);

    // Log the error data to a file
    if (outfile_.is_open()) {
        outfile_ << difference.force.x << "," << difference.force.y << "," << difference.force.z << ","
                 << difference.torque.x << "," << difference.torque.y << "," << difference.torque.z << "\n";
    } else {
        ROS_ERROR("Unable to open file for writing");
    }

    // Optionally reset flags after computation
    wrench_received_ = false;
    predicted_wrench_received_ = false;
}