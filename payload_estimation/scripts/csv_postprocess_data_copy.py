#!/usr/bin/env python3

import pandas as pd
from sklearn.preprocessing import StandardScaler, OneHotEncoder
from sklearn.model_selection import train_test_split

# Load the CSV file
df = pd.read_csv('/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/raw/csv/output.csv')

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

df_filtered = df.loc[~df['Joint Name'].isin(excluded_joints)].copy()

# Save the processed data
df_filtered.to_csv('/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/processed/csv/output.csv', index=False)


# Keep only the first occurrence of consecutive 'force/torque' and drop the rest
# Step 4: Remove consecutive 'force/torque' rows
# Shift the 'Joint Name' column by 1 and compare with the original to find consecutive duplicates
# consecutive_force_torque = (df_filtered['Joint Name'] == 'force/torque') & (df_filtered['Joint Name'].shift() == 'force/torque')
# df_filtered = df_filtered.loc[~consecutive_force_torque]


# Step 2: Handle missing values - only fill numeric columns with their mean
# numeric_cols = df_filtered.select_dtypes(include=['float64', 'int64']).columns
# df_filtered[numeric_cols] = df_filtered[numeric_cols].fillna(df_filtered[numeric_cols].mean())

# Step 3: Convert 'Joint Name' to string (ensure consistent data type for OneHotEncoder)
# df_filtered['Joint Name'] = df_filtered['Joint Name'].astype(str)

# Step 4: Separate features and target columns
# features = df_filtered[['Time', 'Joint Name', 'Position', 'Velocity', 'Effort']].copy()
# targets = df_filtered[['Force X', 'Force Y', 'Force Z', 'Torque X', 'Torque Y', 'Torque Z']].copy()

# Step 5: Set all negative values to 0 (only for numeric columns)
# numeric_feature_cols = ['Time', 'Position', 'Velocity', 'Effort']
# features[numeric_feature_cols] = features[numeric_feature_cols].clip(lower=0)
# targets = targets.clip(lower=0)

# Step 6: One-hot encode the 'Joint Name' column
# encoder = OneHotEncoder(drop='first')  # drop='first' avoids the dummy variable trap
# one_hot_encoded = encoder.fit_transform(features[['Joint Name']]).toarray()
# one_hot_columns = encoder.get_feature_names_out(['Joint Name'])

# Combine the one-hot encoded joint names with the rest of the features
# features = pd.concat([features.drop('Joint Name', axis=1), pd.DataFrame(one_hot_encoded, columns=one_hot_columns)], axis=1)

# Step 7: Ensure both features and targets are filtered based on valid target rows (commented out)
# # Filter valid rows where at least one target value is non-zero
# valid_rows = (targets != 0).any(axis=1)  # Only keep rows where at least one target value is non-zero

# # Use .loc to filter both features and targets, aligning their indices correctly
# features = features.loc[valid_rows].reset_index(drop=True)
# targets = targets.loc[valid_rows].reset_index(drop=True)

# Ensure both features and targets have the same number of rows
# assert len(features) == len(targets), f"Mismatch between feature and target row counts! Features: {len(features)}, Targets: {len(targets)}"

# Step 8: Standardize the continuous features (Time, Position, Velocity, Effort)
# scaler = StandardScaler()
# features[numeric_feature_cols] = scaler.fit_transform(features[numeric_feature_cols])

# Step 9: Train/test split
# X_train, X_test, y_train, y_test = train_test_split(features, targets, test_size=0.2, random_state=42)

# Save the processed data for training
# df_train = pd.DataFrame(X_train, columns=features.columns)
# df_train[['Force X', 'Force Y', 'Force Z', 'Torque X', 'Torque Y', 'Torque Z']] = y_train
# df_train.to_csv('/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/processed/csv/train_data_with_joint_names.csv', index=False)

# Save the test data for future validation
# df_test = pd.DataFrame(X_test, columns=features.columns)
# df_test[['Force X', 'Force Y', 'Force Z', 'Torque X', 'Torque Y', 'Torque Z']] = y_test
# df_test.to_csv('/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/processed/csv/test_data_with_joint_names.csv', index=False)

print("Preprocessing completed with standardization. Ready for training.")