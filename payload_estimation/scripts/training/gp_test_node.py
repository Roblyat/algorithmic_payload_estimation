#!/usr/bin/env python3

import rospy
import pandas as pd
import numpy as np
import pickle
import os
import GPy
from sklearn.metrics import mean_squared_error, mean_absolute_error, r2_score
import json

def load_test_data(test_csv, data_type):
    """
    Loads the test data from a CSV file.
    Assumes the last 6 columns are the target (Effort or Force/Torque), and the rest are the features.
    """
    df_test = pd.read_csv(test_csv)

    # Split features and targets
    X_test = df_test.iloc[:, :-6].values  # Features: all columns except the last 6
    Y_test = df_test.iloc[:, -6:].values  # Targets: last 6 columns (Efforts or Force/Torque depending on data_type)
    
    return X_test, Y_test

def load_gp_model(model_filename):
    """
    Load the trained GP model using pickle.
    """
    rospy.loginfo(f"Loading GP model from {model_filename}")
    with open(model_filename, 'rb') as file:
        gp_model = pickle.load(file)

    return gp_model


def predict_with_gp(gp_model, X_test):
    """
    Make predictions using the trained GP model on the test data.
    - X_test: The feature inputs from the test dataset.
    
    Returns the predicted values and the variances.
    """
    Y_pred, Y_var = gp_model.predict(X_test)

    # Debug: print or log the shapes
    rospy.loginfo(f"Y_pred shape: {Y_pred.shape}, Y_var shape: {Y_var.shape}")
    
    return Y_pred, Y_var

def gp_test_node():
    """
    ROS node for testing the Gaussian Process model using a test dataset (wrench or effort).
    """
    # Initialize the ROS node
    rospy.init_node('gp_test_node')

    # Get the data type (wrench or effort) from ROS parameters
    data_type = rospy.get_param('/rosparam/data_type', 'wrench')  # Default to 'effort'

    # Get the base paths and file names
    if data_type == 'wrench':
        input_folder = os.getenv("ROS_PROCESSED_PATH", "/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/processed/wrench")
        # input_folder = '/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/processed/wrench'
    else:
        input_folder = os.getenv("ROS_PROCESSED_PATH", "/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/processed/effort")
        # input_folder = '/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/processed/effort'
    
    #rosbag name parameter
    rosbag_name = rospy.get_param('/rosparam/rosbag_name', 'recorded_data.bag')
    rosbag_base_name = os.path.splitext(rosbag_name)[0]

    # Depending on the data type, set the test_csv_name and columns appropriately
    if data_type == 'wrench':
        test_csv_param = '_wrench_test_data.csv'
        columns = ['Actual Force X', 'Actual Force Y', 'Actual Force Z',
                   'Actual Torque X', 'Actual Torque Y', 'Actual Torque Z',
                   'Predicted Force X', 'Predicted Force Y', 'Predicted Force Z',
                   'Predicted Torque X', 'Predicted Torque Y', 'Predicted Torque Z',
                   'Variance']  # Column names for wrench data
        output_folder = os.getenv("ROS_RESULTS_PATH", "/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/result/wrench")
        # output_folder = '/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/result/wrench'
    else:
        test_csv_param = '_effort_test_data.csv'
        columns = [f'Actual Effort Joint {i+1}' for i in range(6)] + \
                  [f'Predicted Effort Joint {i+1}' for i in range(6)] + \
                  ['Variance']  # Column names for effort data
        output_folder = os.getenv("ROS_RESULTS_PATH", "/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/result/effort")
        # output_folder = '/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/result/effort'

    # Combine the base name with the test_csv_param
    test_csv_name = f"{rosbag_base_name}{test_csv_param}"
    test_csv = os.path.join(input_folder, test_csv_name)

    # Load the K-Fold parameter as a boolean (default is False)
    use_kfold = rospy.get_param('/rosparam/use_kfold', False)  # Default is False
    use_sparse = rospy.get_param('/rosparam/use_sparse', False)  # Default is False

    # Model output path, depending on data_type (effort or wrench)
    base_model_output_path = os.getenv("ROS_GP_MODELS_PATH", "/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/gp_models")
    model_output_path = os.path.join(base_model_output_path, data_type)
    # model_output_path = os.path.join('/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/gp_models', data_type)

    # Construct the model filename based on the parameters
    suffix = ""
    if use_kfold and use_sparse:
        suffix = "_k_s"
    elif use_kfold:
        suffix = "_k"
    elif use_sparse:
        suffix = "_s"

    # Combine the output path and model name
    model_filename = os.path.join(model_output_path, f"{rosbag_base_name}_{data_type}{suffix}_model.pkl")
    results_csv = os.path.join(output_folder, f"{rosbag_base_name}_{data_type}{suffix}_results.csv")

    # Load the test data
    rospy.loginfo(f"Loading {data_type} test data from {test_csv}")
    X_test, Y_test = load_test_data(test_csv, data_type)

    # Load the GP model
    rospy.loginfo(f"Loading {data_type} GP model from {model_filename}")
    gp_model = load_gp_model(model_filename)

    # Make predictions on the test data
    rospy.loginfo(f"Making predictions on the {data_type} test dataset...")
    Y_pred, Y_var = predict_with_gp(gp_model, X_test)

        # Calculate the test metrics
    mse = mean_squared_error(Y_test, Y_pred)
    rmse = np.sqrt(mse)  # Root Mean Squared Error
    mae = mean_absolute_error(Y_test, Y_pred)
    r2 = r2_score(Y_test, Y_pred)

    # Print the test metrics in the terminal
    rospy.loginfo(f"Test Metrics: MSE={mse}, RMSE={rmse}, MAE={mae}, R^2={r2}")

    # Save the actual vs predicted results to a CSV file
    rospy.loginfo(f"Saving predictions and actual values to {results_csv}")
    
    # Concatenate the test data, predicted values, and variance (for wrench or effort)
    df_results = pd.DataFrame(np.hstack((Y_test, Y_pred, Y_var)), columns=columns)

    df_results.to_csv(results_csv, index=False)
    rospy.loginfo(f"Results saved to {results_csv}")

if __name__ == '__main__':
    try:
        gp_test_node()
    except rospy.ROSInterruptException:
        pass