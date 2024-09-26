#ifndef FRONTEND_H
#define FRONTEND_H

// Standard Libraries
#include "imgui.h"
#include <ros/ros.h>

// Custom Classes
#include "Control.h"
// #include "Measurement_Panel.h"
// #include "Plot_Panel.h"

class Frontend
{
public:
    Frontend(ros::NodeHandle &nh);
    void render();

private:
    Control controlPanel;
    // MeasurementPanel measurementPanel;
    // PlotPanel plotPanel;
    int currentTab;
};

#endif // FRONTEND_H