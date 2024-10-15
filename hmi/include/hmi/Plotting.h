#ifndef PLOTTING_H
#define PLOTTING_H

#include "ros/ros.h"
#include "geometry_msgs/WrenchStamped.h"
#include "imgui.h"
#include "implot.h"
#include <thread>
#include <mutex>
#include <vector>
#include <atomic>

class Plotting {
public:
    Plotting();         // Constructor
    ~Plotting();        // Destructor

    void startPlotting();  // Start live plotting on a separate thread
    void stopPlotting();   // Stop live plotting
    void renderPlot();     // Render the ImGui/ImPlot UI for plotting
    
    std::atomic<bool> plot_running;   // Flag to control the thread loop

private:
    // Worker method that runs in a separate thread to handle live plotting
    void plotWorker();

    // ROS Callback functions for subscribing to topics
    void wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
    void predictedWrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);

    // Thread and control variables
    std::thread plot_thread;          // Thread for the plot worker

    std::mutex data_mutex;            // Mutex for thread-safe data handling
    std::vector<float> time_series;   // Time axis for the plot

    // Data containers for storing wrench and predicted wrench values
    std::vector<float> wrench_force_x, wrench_force_y, wrench_force_z;
    std::vector<float> wrench_torque_x, wrench_torque_y, wrench_torque_z;
    std::vector<float> predicted_force_x, predicted_force_y, predicted_force_z;
    std::vector<float> predicted_torque_x, predicted_torque_y, predicted_torque_z;

    ros::Subscriber wrench_sub;            // ROS subscriber for /wrench
    ros::Subscriber predicted_wrench_sub;  // ROS subscriber for /predicted_wrench
    ros::NodeHandle nh;                    // ROS node handle
};

#endif // PLOTTING_H