#!/usr/bin/env python3

import rospy
import os
import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
import pickle

# Initialize the ROS node first
rospy.init_node('effort_data_preprocessing_node')

# Get the data type (wrench or effort) from ROS parameters
data_type = rospy.get_param('/rosparam/data_type', 'effort')  # Default to 'effort'

# Check if the data_type is 'effort', if not, kill the script
if data_type != 'effort':
    rospy.logerr(f"Data type is '{data_type}'. This script only supports 'effort'. Shutting down.")
    rospy.signal_shutdown("Unsupported data type")
    exit()


def process_joint_states(joint_states_csv, output_csv):
    # Load the joint states CSV file
    df_joint_states = pd.read_csv(joint_states_csv)

    # Step 1: Remove unnecessary joints
    excluded_joints = [
        'gripper_finger_joint', 
        'gripper_robotiq_85_left_finger_tip_joint', 
        'gripper_robotiq_85_left_inner_knuckle_joint',
        'gripper_robotiq_85_right_finger_tip_joint',
        'gripper_robotiq_85_right_inner_knuckle_joint',
        'gripper_robotiq_85_right_knuckle_joint',
        'sensor_measurment_joint'
    ]

    # Filter out excluded joints
    df_joint_states_filtered = df_joint_states.loc[~df_joint_states['Joint Name'].isin(excluded_joints)].copy()

    # Step 2: Group joint states by timestamp (block them by 'Time')
    grouped_joint_states = df_joint_states_filtered.groupby('Time')

    # Prepare the final DataFrame to store joint states with computed acceleration
    combined_data = []

    # Step 3: Compute the acceleration for each row based on the velocity difference between time steps
    time_step = 1 / 40  # Assuming the time step is 0.025 seconds (40Hz)

    # Iterate over the grouped timestamps and joints
    for time, group in grouped_joint_states:
        # Sort by Joint Name to ensure consistency
        group = group.sort_values(by='Joint Name').reset_index(drop=True)
        
        # For each joint at this timestamp, calculate the acceleration (velocity difference)
        for i in range(len(group)):
            if i > 0:  # Skip the first row (as there is no previous time to compare)
                acceleration = (group.iloc[i]['Velocity'] - group.iloc[i-1]['Velocity']) / time_step
            else:
                acceleration = group.iloc[i]['Velocity'] / time_step  # For the first entry, use velocity/time

            # Append the new data (with calculated acceleration)
            combined_data.append([
                group.iloc[i]['Time'],           # Time
                group.iloc[i]['Joint Name'],     # Joint Name
                group.iloc[i]['Position'],       # Position
                group.iloc[i]['Velocity'],       # Velocity
                acceleration,                    # Calculated Acceleration
                group.iloc[i]['Effort']          # Effort
            ])

    # Step 4: Define column names for the new CSV
    combined_columns = ['Time', 'Joint Name', 'Position', 'Velocity', 'Acceleration', 'Effort']

    # Step 5: Create a DataFrame from the combined data and save it to CSV
    df_combined = pd.DataFrame(combined_data, columns=combined_columns)
    df_combined.to_csv(output_csv, index=False)

    print(f"Joint states with computed accelerations saved to {output_csv}")


def prepare_training_data(combined_csv, output_csv):
    # Load the combined CSV file (with acceleration)
    df_combined = pd.read_csv(combined_csv)

    # Prepare the final DataFrame for training
    training_data = []

    # Step 1: Iterate over the combined data
    for i in range(0, len(df_combined) - 5, 6):  # Shift by 6 to ensure correct joint block
        # Get the current block of 6 rows (corresponding to 6 consecutive joint states)
        joint_block = df_combined.iloc[i:i+6]

        # Prepare features (Position, Velocity, Acceleration) and targets (Effort) separately
        joint_features = []
        joint_efforts = []

        for _, joint_row in joint_block.iterrows():
            joint_features.extend([joint_row['Position'], joint_row['Velocity'], joint_row['Acceleration']])
            joint_efforts.append(joint_row['Effort'])

        # Add to training data (flattened joint states followed by efforts)
        training_data.append(joint_features + joint_efforts)


    # Step 2: Define column names for features and targets
    joint_feature_columns = [f'Joint_{i+1}_{param}' for i in range(6) for param in ['Position', 'Velocity', 'Acceleration']]
    effort_columns = [f'Joint_{i+1}_Effort' for i in range(6)]
    columns = joint_feature_columns + effort_columns

    # Step 3: Create DataFrame and save as CSV
    df_training = pd.DataFrame(training_data, columns=columns)
    df_training.to_csv(output_csv, index=False)

    print(f"Training data saved to {output_csv}")


# Method to split and standardize the data
def split_and_standardize(training_csv, train_output_csv, test_output_csv):
    # Load the training data
    df_training = pd.read_csv(training_csv)
    
    # Split into features and targets
    X = df_training.iloc[:, :-6]  # Features: all columns except the last 6
    y = df_training.iloc[:, -6:]  # Targets: the last 6 columns (Force/Torque)

    # Split the data into 80% training and 20% testing
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

    # Standardize the features using StandardScaler
    X_scaler = StandardScaler()


    print(f"Mean before TRAIN scaling: {np.mean(X_train, axis=0)}")  # Should be approximately 0
    print(f"Standard deviation before scaling TRAIN: {np.std(X_train, axis=0)}")  # Should be approximately 1

    X_train_scaled = X_scaler.fit_transform(X_train)

    # # After scaling, check the mean and stddev of the scaled training data
    print(f"Mean after scaling TRAIN: {np.mean(X_train_scaled, axis=0)}")  # Should be approximately 0
    print(f"Standard deviation after scaling TRAIN: {np.std(X_train_scaled, axis=0)}")  # Should be approximately 1

    print(f"Mean before TEST scaling: {np.mean(X_test, axis=0)}")  # Should be approximately 0
    print(f"Standard deviation before scaling TEST: {np.std(X_test, axis=0)}")  # Should be approximately 1
    X_test_scaled = X_scaler.transform(X_test)
    print(f"Mean after TEST scaling: {np.mean(X_test_scaled, axis=0)}")  # Should be approximately 0
    print(f"Standard deviation after scaling TEST: {np.std(X_test_scaled, axis=0)}")  # Should be approximately 1

    # Standardize the targets using a separate StandardScaler
    y_scaler = StandardScaler()
    y_train_scaled = y_scaler.fit_transform(y_train)
    y_test_scaled = y_scaler.transform(y_test)

    # Save both scalers and their corresponding feature/target names to files
    X_scaler_data = {'scaler': X_scaler, 'columns': X.columns.tolist()}
    y_scaler_data = {'scaler': y_scaler, 'columns': y.columns.tolist()}

    with open(X_scaler_filename, 'wb') as f:
        pickle.dump(X_scaler_data, f)
    
    with open(y_scaler_filename, 'wb') as f:
        pickle.dump(y_scaler_data, f)

    # Recombine features and scaled targets for both train and test
    df_train = pd.DataFrame(X_train_scaled, columns=X.columns)
    df_train = pd.concat([df_train, pd.DataFrame(y_train_scaled, columns=y.columns)], axis=1)
    
    df_test = pd.DataFrame(X_test_scaled, columns=X.columns)
    df_test = pd.concat([df_test, pd.DataFrame(y_test_scaled, columns=y.columns)], axis=1)

    # Save the training and testing datasets to CSV
    df_train.to_csv(train_output_csv, index=False)
    df_test.to_csv(test_output_csv, index=False)

    print(f"Train data saved to {train_output_csv}")
    print(f"Test data saved to {test_output_csv}")


if __name__ == "__main__":
    
    # Paths to your CSV files
    output_folder = '/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/processed/effort'
    raw_csv_folder = '/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/raw/csv/train'
    rosbag_name = rospy.get_param('/rosparam/rosbag_name', 'recorded_data.bag')

    rosbag_base_name = os.path.splitext(rosbag_name)[0]
    joint_states_csv = os.path.join(raw_csv_folder, f"{rosbag_base_name}_jointstates.csv")
    combined_csv = os.path.join(output_folder, f"{rosbag_base_name}_effort_combined.csv") 
    raw_training_csv = os.path.join(output_folder, f"{rosbag_base_name}_effort_raw_training.csv")

    train_data_csv = os.path.join(output_folder, f"{rosbag_base_name}_effort_train_data.csv")

    test_data_csv = os.path.join(output_folder, f"{rosbag_base_name}_effort_test_data.csv")  # Corrected: use test_csv_name

    # Folder path for saving the scaler
    scaler_output_path = '/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/gp_models/scalers/effort'

    # Construct the scaler file name conditionally
    X_scaler_filename = os.path.join(scaler_output_path, f"{rosbag_base_name}_effort_feature_scaler.pkl")

    y_scaler_filename = os.path.join(scaler_output_path, f"{rosbag_base_name}_effort_target_scaler.pkl")

    # Step 1: Process joint states and wrench data
    process_joint_states(joint_states_csv, combined_csv)

    # Step 2: Prepare the training data
    prepare_training_data(combined_csv, raw_training_csv)

    # Step 3: Split and standardize the data
    split_and_standardize(raw_training_csv, train_data_csv, test_data_csv)