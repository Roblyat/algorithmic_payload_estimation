#ifndef PLOTTING_H
#define PLOTTING_H

#include <ros/ros.h>
#include "imgui.h"

// Plotting class handles rendering of plots in the GUI
class Plotting {
public:
    // Method to render a placeholder for future plot functionality
    Plotting(ros::NodeHandle& N);
    void renderPlot();

private:
    ros::NodeHandle nh;
};

#endif // PLOTTING_H
