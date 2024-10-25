#!/usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
import pandas as pd
import os

# Get ROS parameters
rosbag_name = rospy.get_param('/rosparam/rosbag_name', 'recorded_data.bag')
data_type = rospy.get_param('/rosparam/data_type', 'wrench')  # Default to 'wrench'
rosbag_base_name = os.path.splitext(rosbag_name)[0]

# Load CSV data for wrench and predicted_wrench
wrench_df = pd.read_csv(f'/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/raw/csv/test/{rosbag_base_name}_wrench.csv')
predicted_wrench_df = pd.read_csv(f'/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/raw/csv/test/{rosbag_base_name}_predicted_wrench.csv')

# Directory to save the plots
save_directory = '/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/result/live'
os.makedirs(save_directory, exist_ok=True)

# Plot comparison for each axis of force and torque (X, Y, Z)

# Force X comparison
plt.figure(figsize=(10, 6))
plt.plot(wrench_df['Time'].values, wrench_df['Force X'].values, label='Actual Force X', color='green')
plt.plot(predicted_wrench_df['Time'].values, predicted_wrench_df['Predicted Force X'].values, label='Predicted Force X', color='orange', linestyle='dashed')
plt.title('Force X: Actual vs Predicted')
plt.xlabel('Time (s)')
plt.ylabel('Force X')
plt.legend()
plt.grid(True)
plt.savefig(os.path.join(save_directory, f'{rosbag_base_name}_{data_type}_force_x.png'))
rospy.loginfo(f"Force X plot saved as {rosbag_base_name}_{data_type}_force_x.png")
plt.show()

# Force Y comparison
plt.figure(figsize=(10, 6))
plt.plot(wrench_df['Time'].values, wrench_df['Force Y'].values, label='Actual Force Y', color='blue')
plt.plot(predicted_wrench_df['Time'].values, predicted_wrench_df['Predicted Force Y'].values, label='Predicted Force Y', color='red', linestyle='dashed')
plt.title('Force Y: Actual vs Predicted')
plt.xlabel('Time (s)')
plt.ylabel('Force Y')
plt.legend()
plt.grid(True)
plt.savefig(os.path.join(save_directory, f'{rosbag_base_name}_{data_type}_force_y.png'))
rospy.loginfo(f"Force Y plot saved as {rosbag_base_name}_{data_type}_force_y.png")
plt.show()

# Force Z comparison
plt.figure(figsize=(10, 6))
plt.plot(wrench_df['Time'].values, wrench_df['Force Z'].values, label='Actual Force Z', color='purple')
plt.plot(predicted_wrench_df['Time'].values, predicted_wrench_df['Predicted Force Z'].values, label='Predicted Force Z', color='yellow', linestyle='dashed')
plt.title('Force Z: Actual vs Predicted')
plt.xlabel('Time (s)')
plt.ylabel('Force Z')
plt.legend()
plt.grid(True)
plt.savefig(os.path.join(save_directory, f'{rosbag_base_name}_{data_type}_force_z.png'))
rospy.loginfo(f"Force Z plot saved as {rosbag_base_name}_{data_type}_force_z.png")
plt.show()

# Torque X comparison
plt.figure(figsize=(10, 6))
plt.plot(wrench_df['Time'].values, wrench_df['Torque X'].values, label='Actual Torque X', color='cyan')
plt.plot(predicted_wrench_df['Time'].values, predicted_wrench_df['Predicted Torque X'].values, label='Predicted Torque X', color='orange', linestyle='dashed')
plt.title('Torque X: Actual vs Predicted')
plt.xlabel('Time (s)')
plt.ylabel('Torque X')
plt.legend()
plt.grid(True)
plt.savefig(os.path.join(save_directory, f'{rosbag_base_name}_{data_type}_torque_x.png'))
rospy.loginfo(f"Torque X plot saved as {rosbag_base_name}_{data_type}_torque_x.png")
plt.show()

# Torque Y comparison
plt.figure(figsize=(10, 6))
plt.plot(wrench_df['Time'].values, wrench_df['Torque Y'].values, label='Actual Torque Y', color='red')
plt.plot(predicted_wrench_df['Time'].values, predicted_wrench_df['Predicted Torque Y'].values, label='Predicted Torque Y', color='blue', linestyle='dashed')
plt.title('Torque Y: Actual vs Predicted')
plt.xlabel('Time (s)')
plt.ylabel('Torque Y')
plt.legend()
plt.grid(True)
plt.savefig(os.path.join(save_directory, f'{rosbag_base_name}_{data_type}_torque_y.png'))
rospy.loginfo(f"Torque Y plot saved as {rosbag_base_name}_{data_type}_torque_y.png")
plt.show()

# Torque Z comparison
plt.figure(figsize=(10, 6))
plt.plot(wrench_df['Time'].values, wrench_df['Torque Z'].values, label='Actual Torque Z', color='magenta')
plt.plot(predicted_wrench_df['Time'].values, predicted_wrench_df['Predicted Torque Z'].values, label='Predicted Torque Z', color='gray', linestyle='dashed')
plt.title('Torque Z: Actual vs Predicted')
plt.xlabel('Time (s)')
plt.ylabel('Torque Z')
plt.legend()
plt.grid(True)
plt.savefig(os.path.join(save_directory, f'{rosbag_base_name}_{data_type}_torque_z.png'))
rospy.loginfo(f"Torque Z plot saved as {rosbag_base_name}_{data_type}_torque_z.png")
plt.show()