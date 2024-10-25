#include "ftExtractor.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ft_extractor_node");
    ros::NodeHandle nh;

    ftExtractor extractor(nh);  // Create an instance of ftExtractor

    ros::spin();  // Keep the node running
    return 0;
}
