#!/usr/bin/env python3

import matplotlib.pyplot as plt
import pandas as pd

# Load CSV data for joint_states and predicted_effort
joint_states_df = pd.read_csv('/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/raw/csv/test/cartesian_04x3_jointstates.csv')
predicted_effort_df = pd.read_csv('/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/raw/csv/test/cartesian_04x3_predicted_effort.csv')

# Correct joint to motor mapping for UR5
motor_joints = {
    'Motor 1': 'ur5_elbow_joint',
    'Motor 2': 'ur5_shoulder_lift_joint',
    'Motor 3': 'ur5_shoulder_pan_joint',
    'Motor 4': 'ur5_wrist_1_joint',
    'Motor 5': 'ur5_wrist_2_joint',
    'Motor 6': 'ur5_wrist_3_joint'
}

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
    plt.show()
