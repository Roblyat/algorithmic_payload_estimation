#!/usr/bin/env python3

import rospy
import os
import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
import pickle


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
    grouped_joint_states = df_joint_states_filtered.groupby('Joint Name')

    # Prepare the final DataFrame to store joint states with computed acceleration
    combined_data = []

    # Step 3: For each joint, compute the acceleration by dividing velocity by 0.025 (time step for 40Hz)
    time_step = 1 / 40  # 0.025 seconds
    for joint_name, joint_data in grouped_joint_states:
        # Iterate through each row to compute acceleration
        for _, row in joint_data.iterrows():
            acceleration = row['Velocity'] / time_step  # Velocity divided by 0.025

            # Append the new data (with acceleration) to the combined_data list
            combined_data.append([
                row['Time'],           # Time
                row['Joint Name'],     # Joint Name
                row['Position'],       # Position
                row['Velocity'],       # Velocity
                row['Effort'],         # Effort
                acceleration           # Calculated Acceleration
            ])

    # Step 4: Define column names for the new CSV
    combined_columns = ['Time', 'Joint Name', 'Position', 'Velocity', 'Effort', 'Acceleration']

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
    for i in range(len(df_combined) - 5):  # Adjust the range to avoid index errors
        # Get the current block of 6 rows (corresponding to 6 consecutive joint states)
        joint_block = df_combined.iloc[i:i+6]

        # Prepare features (Position, Velocity, Acceleration) and targets (Effort) separately
        joint_features = []
        joint_efforts = []

        for _, joint_row in joint_block.iterrows():
            # Append the features (Position, Velocity, Acceleration)
            joint_features.extend([joint_row['Position'], joint_row['Velocity'], joint_row['Acceleration']])
            # Collect the efforts separately (targets)
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


# New method to split and standardize the data
def split_and_standardize(training_csv, train_output_csv, test_output_csv):
    # Load the training data
    df_training = pd.read_csv(training_csv)
    
    # Split into features and targets
    X = df_training.iloc[:, :-6]  # Features: all columns except the last 6
    y = df_training.iloc[:, -6:]  # Targets: the last 6 columns (Force/Torque)

    # Split the data into 80% training and 20% testing
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

    # Standardize the features using StandardScaler
    scaler = StandardScaler()

    # Fit on training data and transform the training data
    X_train_scaled = scaler.fit_transform(X_train)

    # Transform the test data using the same scaler (important)
    X_test_scaled = scaler.transform(X_test)

    # Save both the scaler and the feature names to a file using pickle
    scaler_data = {
        'scaler': scaler,
        'columns': X.columns.tolist()  # Save feature names as a list
    }

    with open(scaler_filename, 'wb') as f:
        pickle.dump(scaler_data, f)

    # Recombine features and targets for both train and test
    df_train = pd.DataFrame(X_train_scaled, columns=X.columns)
    df_train = pd.concat([df_train, y_train.reset_index(drop=True)], axis=1)
    
    df_test = pd.DataFrame(X_test_scaled, columns=X.columns)
    df_test = pd.concat([df_test, y_test.reset_index(drop=True)], axis=1)

    # Save the training and testing datasets to CSV
    df_train.to_csv(train_output_csv, index=False)
    df_test.to_csv(test_output_csv, index=False)

    print(f"Train data saved to {train_output_csv}")
    print(f"Test data saved to {test_output_csv}")

if __name__ == "__main__":
    
    # Paths to your CSV files
    output_folder = rospy.get_param('/rosparam/preprocessed_csv_path', '/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/processed') 
    # Folder path for joint states and wrench CSVs
    raw_csv_folder = '/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/raw/csv'
    rosbag_name = rospy.get_param('/rosparam/rosbag_name', 'recorded_data.bag')

    rosbag_base_name = os.path.splitext(rosbag_name)[0]
    joint_states_csv = os.path.join(raw_csv_folder, f"{rosbag_base_name}_jointstates.csv")
    combined_csv = os.path.join(output_folder, f"{rosbag_base_name}_effort_combined.csv") #Matched the closest timestamp of wrench data to joint states block (6 joints)
    raw_training_csv = os.path.join(output_folder, f"{rosbag_base_name}_effort_raw_training.csv") #Processed joint states and wrench data

    train_csv_name = rospy.get_param('/rosparam/train_csv_name', '_effort_train_data.csv')
    train_data_csv = os.path.join(output_folder, f"{rosbag_base_name}{train_csv_name}")

    test_csv_name = rospy.get_param('/rosparam/test_csv_name', '_effort_test_data.csv')
    test_data_csv = os.path.join(output_folder, f"{rosbag_base_name}{test_csv_name}")  # Corrected: use test_csv_name

    # Folder path for saving the scaler
    scaler_output_path = '/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/gp_models/scalers/effort'
    scaler_filename = os.path.join(scaler_output_path, f"{rosbag_base_name}_effort_scaler.pkl")

    # Step 1: Process joint states and wrench data
    process_joint_states(joint_states_csv, combined_csv)

    # Step 2: Prepare the training data
    prepare_training_data(combined_csv, raw_training_csv)

    # Step 3: Split and standardize the data
    split_and_standardize(raw_training_csv, train_data_csv, test_data_csv)