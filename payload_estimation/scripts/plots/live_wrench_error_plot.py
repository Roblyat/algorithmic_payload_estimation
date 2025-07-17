#!/usr/bin/env python3

import rospy
import pandas as pd
import matplotlib.pyplot as plt
import os

# Initialize the ROS node
rospy.init_node('wrench_error_plot_node')

# Get rosbag_name and rosbag_base_name from ROS parameter server
rosbag_name = rospy.get_param('/rosparam/rosbag_name', 'recorded_data.bag')
rosbag_base_name = os.path.splitext(rosbag_name)[0]
data_type = "wrench"  # Set to "wrench" since this is a wrench plot

# Load the error CSV file based on rosbag_base_name
file_path = f"/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/raw/csv/test/{rosbag_base_name}_error.csv"
data = pd.read_csv(file_path)

# Define the save directory for the plots
save_directory = '/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/result/live'
os.makedirs(save_directory, exist_ok=True)

# Plot Force Errors
plt.figure()
plt.plot(data['force_x_error'], label='Force X Error')
plt.plot(data['force_y_error'], label='Force Y Error')
plt.plot(data['force_z_error'], label='Force Z Error')
plt.title('Force Errors over Time')
plt.xlabel('Time Step')
plt.ylabel('Force Error (N)')
plt.legend()
force_error_plot_path = os.path.join(save_directory, f'{rosbag_base_name}_{data_type}_force_error_plot.png')
plt.savefig(force_error_plot_path)
rospy.loginfo(f"Force error plot saved at: {force_error_plot_path}")

# Plot Torque Errors
plt.figure()
plt.plot(data['torque_x_error'], label='Torque X Error')
plt.plot(data['torque_y_error'], label='Torque Y Error')
plt.plot(data['torque_z_error'], label='Torque Z Error')
plt.title('Torque Errors over Time')
plt.xlabel('Time Step')
plt.ylabel('Torque Error (Nm)')
plt.legend()
torque_error_plot_path = os.path.join(save_directory, f'{rosbag_base_name}_{data_type}_torque_error_plot.png')
plt.savefig(torque_error_plot_path)
rospy.loginfo(f"Torque error plot saved at: {torque_error_plot_path}")

# Show plots if needed
plt.show()