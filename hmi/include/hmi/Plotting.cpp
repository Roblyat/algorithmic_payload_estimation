#include "Plotting.h"
#include "geometry_msgs/WrenchStamped.h"
#include "ros/ros.h"
#include "implot.h"
#include <chrono>
#include <thread>
#include <algorithm>

// Constructor
Plotting::Plotting() : plot_running(false) {
    // Initialize ROS subscribers
    wrench_sub = nh.subscribe("/wrench", 10, &Plotting::wrenchCallback, this);
    predicted_wrench_sub = nh.subscribe("/predicted_wrench", 10, &Plotting::predictedWrenchCallback, this);
}

// Destructor
Plotting::~Plotting() {
    stopPlotting();  // Ensure the plotting is stopped when the object is destroyed
}

// Start live plotting in a separate thread
void Plotting::startPlotting() {
    if (plot_running.load()) {
        return;  // Already running
    }
    plot_running.store(true);
    plot_thread = std::thread(&Plotting::plotWorker, this);  // Start the worker thread
}

// Stop live plotting
void Plotting::stopPlotting() {
    if (plot_running.load()) {
        plot_running.store(false);  // Set flag to false to stop the worker thread
        if (plot_thread.joinable()) {
            plot_thread.join();  // Wait for the thread to finish
        }
    }
}

// Worker thread for live plotting
void Plotting::plotWorker() {
    while (plot_running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Sleep for a short period

        // Add time axis data
        std::lock_guard<std::mutex> lock(data_mutex);
        float current_time = ros::Time::now().toSec();
        time_series.push_back(current_time);

        // Limit the size of the data vectors to avoid overflow (500 points max)
        const size_t max_size = 50;
        if (time_series.size() > max_size) {
            time_series.erase(time_series.begin());
            wrench_force_x.erase(wrench_force_x.begin());
            wrench_force_y.erase(wrench_force_y.begin());
            wrench_force_z.erase(wrench_force_z.begin());
            wrench_torque_x.erase(wrench_torque_x.begin());
            wrench_torque_y.erase(wrench_torque_y.begin());
            wrench_torque_z.erase(wrench_torque_z.begin());
            predicted_force_x.erase(predicted_force_x.begin());
            predicted_force_y.erase(predicted_force_y.begin());
            predicted_force_z.erase(predicted_force_z.begin());
            predicted_torque_x.erase(predicted_torque_x.begin());
            predicted_torque_y.erase(predicted_torque_y.begin());
            predicted_torque_z.erase(predicted_torque_z.begin());
        }
    }
}

// ROS callback for /wrench
void Plotting::wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex);  // Ensure thread-safe access to data
    wrench_force_x.push_back(msg->wrench.force.x);
    wrench_force_y.push_back(msg->wrench.force.y);
    wrench_force_z.push_back(msg->wrench.force.z);
    wrench_torque_x.push_back(msg->wrench.torque.x);
    wrench_torque_y.push_back(msg->wrench.torque.y);
    wrench_torque_z.push_back(msg->wrench.torque.z);
}

// ROS callback for /predicted_wrench
void Plotting::predictedWrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(data_mutex);  // Ensure thread-safe access to data
    predicted_force_x.push_back(msg->wrench.force.x);
    predicted_force_y.push_back(msg->wrench.force.y);
    predicted_force_z.push_back(msg->wrench.force.z);
    predicted_torque_x.push_back(msg->wrench.torque.x);
    predicted_torque_y.push_back(msg->wrench.torque.y);
    predicted_torque_z.push_back(msg->wrench.torque.z);
}

void Plotting::renderPlot() {
    std::lock_guard<std::mutex> lock(data_mutex);  // Ensure thread-safe access to data

    // Convert vectors to float vectors for ImGui
    std::vector<float> floatTimeSeries(time_series.begin(), time_series.end());

    // Ensure predicted data vectors are not larger than actual data
    std::vector<float> floatPredictedForceX(predicted_force_x.begin(), predicted_force_x.end());
    std::vector<float> floatPredictedForceY(predicted_force_y.begin(), predicted_force_y.end());
    std::vector<float> floatPredictedForceZ(predicted_force_z.begin(), predicted_force_z.end());
    std::vector<float> floatPredictedTorqueX(predicted_torque_x.begin(), predicted_torque_x.end());
    std::vector<float> floatPredictedTorqueY(predicted_torque_y.begin(), predicted_torque_y.end());
    std::vector<float> floatPredictedTorqueZ(predicted_torque_z.begin(), predicted_torque_z.end());

    // Force plot
    if (ImGui::CollapsingHeader("Force")) {
        ImGui::PlotLines("Force X (Wrench)", wrench_force_x.data(), wrench_force_x.size(), 0, nullptr, FLT_MIN, FLT_MAX, ImVec2(0, 80));
        ImGui::PlotLines("Force Y (Wrench)", wrench_force_y.data(), wrench_force_y.size(), 0, nullptr, FLT_MIN, FLT_MAX, ImVec2(0, 80));
        ImGui::PlotLines("Force Z (Wrench)", wrench_force_z.data(), wrench_force_z.size(), 0, nullptr, FLT_MIN, FLT_MAX, ImVec2(0, 80));

        ImGui::PlotLines("Predicted Force X", floatPredictedForceX.data(), floatPredictedForceX.size(), 0, nullptr, FLT_MIN, FLT_MAX, ImVec2(0, 80));
        ImGui::PlotLines("Predicted Force Y", floatPredictedForceY.data(), floatPredictedForceY.size(), 0, nullptr, FLT_MIN, FLT_MAX, ImVec2(0, 80));
        ImGui::PlotLines("Predicted Force Z", floatPredictedForceZ.data(), floatPredictedForceZ.size(), 0, nullptr, FLT_MIN, FLT_MAX, ImVec2(0, 80));
    }

    // Torque plot
    if (ImGui::CollapsingHeader("Torque")) {
        ImGui::PlotLines("Torque X (Wrench)", wrench_torque_x.data(), wrench_torque_x.size(), 0, nullptr, FLT_MIN, FLT_MAX, ImVec2(0, 80));
        ImGui::PlotLines("Torque Y (Wrench)", wrench_torque_y.data(), wrench_torque_y.size(), 0, nullptr, FLT_MIN, FLT_MAX, ImVec2(0, 80));
        ImGui::PlotLines("Torque Z (Wrench)", wrench_torque_z.data(), wrench_torque_z.size(), 0, nullptr, FLT_MIN, FLT_MAX, ImVec2(0, 80));

        ImGui::PlotLines("Predicted Torque X", floatPredictedTorqueX.data(), floatPredictedTorqueX.size(), 0, nullptr, FLT_MIN, FLT_MAX, ImVec2(0, 80));
        ImGui::PlotLines("Predicted Torque Y", floatPredictedTorqueY.data(), floatPredictedTorqueY.size(), 0, nullptr, FLT_MIN, FLT_MAX, ImVec2(0, 80));
        ImGui::PlotLines("Predicted Torque Z", floatPredictedTorqueZ.data(), floatPredictedTorqueZ.size(), 0, nullptr, FLT_MIN, FLT_MAX, ImVec2(0, 80));
    }
}
