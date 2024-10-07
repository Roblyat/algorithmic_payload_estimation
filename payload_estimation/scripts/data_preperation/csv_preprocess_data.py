import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler

def process_joint_states(joint_states_csv, wrench_csv, output_csv):
    # Load the joint states CSV file
    df_joint_states = pd.read_csv(joint_states_csv)

    # Load the wrench CSV file
    df_wrench = pd.read_csv(wrench_csv)

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

    # Prepare the final DataFrame to store combined joint states and wrench data
    combined_data = []

    # Step 3: For each joint states block, find the next nearest wrench data by timestamp (greater than the block time)
    for time, joint_block in grouped_joint_states:
        # Find the next wrench timestamp that is greater than the joint states block time
        wrench_after_block = df_wrench[df_wrench['Time'] > time]
        if not wrench_after_block.empty:
            next_wrench_idx = wrench_after_block['Time'].idxmin()  # Get the nearest greater timestamp
            wrench_row = df_wrench.loc[next_wrench_idx]

            # Add the joint block to the combined data
            for _, joint_row in joint_block.iterrows():
                combined_data.append(list(joint_row))

            # Create a new row for the wrench data with empty position, velocity, effort and add it to the combined data
            wrench_data_row = [
                wrench_row['Time'],  # Time from the wrench data
                'force/torque',  # Joint Name
                '',  # Position
                '',  # Velocity
                '',  # Effort
                wrench_row['Force X'],  # Force X
                wrench_row['Force Y'],  # Force Y
                wrench_row['Force Z'],  # Force Z
                wrench_row['Torque X'],  # Torque X
                wrench_row['Torque Y'],  # Torque Y
                wrench_row['Torque Z']   # Torque Z
            ]
            combined_data.append(wrench_data_row)

    # Step 4: Define column names for the new CSV
    combined_columns = ['Time', 'Joint Name', 'Position', 'Velocity', 'Effort', 
                        'Force X', 'Force Y', 'Force Z', 'Torque X', 'Torque Y', 'Torque Z']

    # Step 5: Create a DataFrame from the combined data and save it to CSV
    df_combined = pd.DataFrame(combined_data, columns=combined_columns)
    df_combined.to_csv(output_csv, index=False)

    print(f"Combined joint states and wrench data saved to {output_csv}")

def prepare_training_data(combined_csv, output_csv):
    # Load the combined CSV file
    df_combined = pd.read_csv(combined_csv)

    # Prepare the final DataFrame for training
    training_data = []

    # Step 1: Iterate over the combined data
    for i in range(len(df_combined)):
        # Check if the current row is a 'force/torque' row
        if df_combined.loc[i, 'Joint Name'] == 'force/torque':
            # Get the previous 6 rows (joint states block)
            joint_block = df_combined.iloc[i-6:i]

            # Flatten the joint states block (6 joints × 3 values)
            joint_features = []
            for _, joint_row in joint_block.iterrows():
                joint_features.extend([joint_row['Position'], joint_row['Velocity'], joint_row['Effort']])

            # Extract the current row's FT values as target (1 × 6 values)
            ft_targets = [
                df_combined.loc[i, 'Force X'], df_combined.loc[i, 'Force Y'], df_combined.loc[i, 'Force Z'], 
                df_combined.loc[i, 'Torque X'], df_combined.loc[i, 'Torque Y'], df_combined.loc[i, 'Torque Z']
            ]

            # Add to training data (flattened joint states and wrench row)
            training_data.append(joint_features + ft_targets)

    # Step 2: Define column names
    joint_columns = [f'Joint_{i+1}_{param}' for i in range(6) for param in ['Position', 'Velocity', 'Effort']]
    ft_columns = ['Force X', 'Force Y', 'Force Z', 'Torque X', 'Torque Y', 'Torque Z']
    columns = joint_columns + ft_columns

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
    joint_states_csv = '/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/raw/csv/joint_states_output.csv'
    wrench_csv = '/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/raw/csv/wrench_output.csv'
    combined_csv = '/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/processed/csv/combined_output.csv'
    training_csv = '/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/processed/csv/training_output.csv'
    train_output_csv = '/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/processed/csv/train_data.csv'
    test_output_csv = '/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/processed/csv/test_data.csv'

    # Step 1: Process joint states and wrench data
    process_joint_states(joint_states_csv, wrench_csv, combined_csv)

    # Step 2: Prepare the training data
    prepare_training_data(combined_csv, training_csv)

    # Step 3: Split and standardize the data
    split_and_standardize(training_csv, train_output_csv, test_output_csv)