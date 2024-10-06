#!/usr/bin/env python3

import pandas as pd
from sklearn.preprocessing import OneHotEncoder
from sklearn.model_selection import train_test_split

# Load the CSV file
df = pd.read_csv('/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/raw/csv/output.csv')

# Step 1: Remove unnecessary joints (already done in previous script)
excluded_joints = [
    'gripper_finger_joint', 
    'gripper_robotiq_85_left_finger_tip_joint', 
    'gripper_robotiq_85_left_inner_knuckle_joint',
    'gripper_robotiq_85_right_finger_tip_joint',
    'gripper_robotiq_85_right_inner_knuckle_joint',
    'gripper_robotiq_85_right_knuckle_joint',
    'sensor_measurment_joint'
]

# Use .loc to avoid SettingWithCopyWarning
df_filtered = df.loc[~df['Joint Name'].isin(excluded_joints)]

# Step 2: Handle missing values (fill with zeros or another strategy)
df_filtered.loc[:, :] = df_filtered.fillna(0)  # Use .loc to avoid the warning

# Step 3: Convert 'Joint Name' to string (to ensure consistent data type for OneHotEncoder)
df_filtered.loc[:, 'Joint Name'] = df_filtered['Joint Name'].astype(str)

# Step 4: Separate features and target columns
features = df_filtered[['Time', 'Joint Name', 'Position', 'Velocity', 'Effort']]
targets = df_filtered[['Force X', 'Force Y', 'Force Z', 'Torque X', 'Torque Y', 'Torque Z']]

# Step 5: Drop rows where targets are all zero (ensures valid data)
valid_rows = (targets != 0).any(axis=1)
features = features[valid_rows]
targets = targets[valid_rows]

# Ensure both features and targets have the same number of rows
assert len(features) == len(targets), "Mismatch between feature and target row counts!"

# Step 6: One-hot encode the 'Joint Name' column
encoder = OneHotEncoder(drop='first')  # drop='first' avoids the dummy variable trap
one_hot_encoded = encoder.fit_transform(features[['Joint Name']]).toarray()
one_hot_columns = encoder.get_feature_names_out(['Joint Name'])

# Step 7: Combine the one-hot encoded joint names with the rest of the features
features = pd.concat([features.drop('Joint Name', axis=1), pd.DataFrame(one_hot_encoded, columns=one_hot_columns)], axis=1)

# Step 8: Train/test split
X_train, X_test, y_train, y_test = train_test_split(features, targets, test_size=0.2, random_state=42)

# Save the processed data
df_train = pd.DataFrame(X_train, columns=features.columns)
df_train[['Force X', 'Force Y', 'Force Z', 'Torque X', 'Torque Y', 'Torque Z']] = y_train
df_train.to_csv('/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/processed/csv/train_data_with_joint_names.csv', index=False)

# Save the test data (for future validation if needed)
df_test = pd.DataFrame(X_test, columns=features.columns)
df_test[['Force X', 'Force Y', 'Force Z', 'Torque X', 'Torque Y', 'Torque Z']] = y_test
df_test.to_csv('/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/processed/csv/test_data_with_joint_names.csv', index=False)

print("Preprocessing completed. Ready for training.")