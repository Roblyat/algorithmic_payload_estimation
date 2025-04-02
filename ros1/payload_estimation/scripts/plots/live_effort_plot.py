#!/usr/bin/env python3
import rospy
import matplotlib.pyplot as plt
import pandas as pd
import os

# Get ROS parameters
rosbag_name = rospy.get_param('/rosparam/rosbag_name', 'recorded_data.bag')
data_type = rospy.get_param('/rosparam/data_type', 'effort')  # Default to 'effort'
rosbag_base_name = os.path.splitext(rosbag_name)[0]

# Load CSV data for joint_states and predicted_effort
joint_states_df = pd.read_csv(f'/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/raw/csv/test/{rosbag_base_name}_jointstates.csv')
predicted_effort_df = pd.read_csv(f'/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/raw/csv/test/{rosbag_base_name}_predicted_effort.csv')

# Correct joint to motor mapping for UR5
motor_joints = {
    'Motor 1': 'ur5_elbow_joint',
    'Motor 2': 'ur5_shoulder_lift_joint',
    'Motor 3': 'ur5_shoulder_pan_joint',
    'Motor 4': 'ur5_wrist_1_joint',
    'Motor 5': 'ur5_wrist_2_joint',
    'Motor 6': 'ur5_wrist_3_joint'
}

# Directory to save the plots
save_directory = '/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/result/live'
os.makedirs(save_directory, exist_ok=True)

# Loop through each motor and plot the actual vs predicted effort
for motor_num, joint_name in motor_joints.items():
    # Filter the joint states for the specific joint
    joint_time = joint_states_df[joint_states_df['Joint Name'] == joint_name]['Time']
    joint_effort = joint_states_df[joint_states_df['Joint Name'] == joint_name]['Effort']

    # Extract the predicted efforts for the corresponding motor
    predicted_time = predicted_effort_df['Time']
    predicted_effort = predicted_effort_df[f'Effort Motor {motor_num.split()[-1]}']

    # Plot the actual vs predicted effort
    plt.figure(figsize=(10, 6))
    plt.plot(joint_time.values, joint_effort.values, label=f'{joint_name} Effort (Actual)', color='blue')
    plt.plot(predicted_time.values, predicted_effort.values, label=f'Motor {motor_num} Predicted Effort', color='red', linestyle='dashed')

    # Customize the plot
    plt.title(f'Effort Comparison: {joint_name} vs Predicted Effort (Motor {motor_num})')
    plt.xlabel('Time (s)')
    plt.ylabel('Effort')
    plt.legend()
    plt.grid(True)

    # Save the plot as a PNG file
    save_path = os.path.join(save_directory, f"{rosbag_base_name}_{data_type}_motor_{motor_num.split()[-1]}.png")
    plt.savefig(save_path)
    rospy.loginfo(f"Plot saved: {save_path}")

    # Show the plot
    plt.show()
