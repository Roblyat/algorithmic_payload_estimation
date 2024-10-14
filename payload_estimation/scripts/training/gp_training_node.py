#!/usr/bin/env python3

import rospy
import pandas as pd
import GPy
import numpy as np
import pickle  # Import pickle for saving/loading models
import os  # For joining paths

def load_training_data(training_csv, subsample_size):
    """
    Loads the training data from a CSV file.
    Assumes the last 6 columns are the target (Force/Torque), and the rest are the features.
    """
    df_training = pd.read_csv(training_csv)

    # If the dataset is larger than subsample_size, randomly select a subset of the rows
    if len(df_training) > subsample_size:
        rospy.loginfo(f"Subsampling {subsample_size} from a dataset of size {len(df_training)}")
        df_training = df_training.sample(n=subsample_size, random_state=42)
    
    # Split features and targets
    X = df_training.iloc[:, :-6].values  # Features: all columns except the last 6
    Y = df_training.iloc[:, -6:].values  # Targets: last 6 columns (Force/Torque)
    
    return X, Y

def train_gp_model(X, Y, kernel=None):
    """
    Train a Gaussian Process (GP) model using GPy library.
    - X: Features (input joint states)
    - Y: Targets (output force/torque)
    - kernel: Optional kernel to use in the GP model. Defaults to RBF kernel.
    
    Returns the trained GP model.
    """
    # Retrieve the kernel type from the ROS parameter server (default to 'RBF')
    kernel_type = rospy.get_param('/rosparam/kernel', 'RBF')

    # Define the kernel based on the parameter or use the provided kernel
    if kernel_type == 'RBF':
        kernel = GPy.kern.RBF(input_dim=X.shape[1], variance=1., lengthscale=1.)
    elif kernel_type == 'Matern52':
        kernel = GPy.kern.Matern52(input_dim=X.shape[1], variance=1., lengthscale=1.)
    elif kernel_type == 'Linear':
        kernel = GPy.kern.Linear(input_dim=X.shape[1])
    else:
        rospy.logwarn(f"Unknown kernel type '{kernel_type}', defaulting to RBF.")
        kernel = GPy.kern.RBF(input_dim=X.shape[1], variance=1., lengthscale=1.)

    # Create the GP regression model
    gp_model = GPy.models.GPRegression(X, Y, kernel)
    
    # Train the model by optimizing the hyperparameters
    gp_model.optimize(messages=True)

    # Return the trained model
    return gp_model

def save_gp_model(gp_model, model_filename):
    """
    Saves the trained GP model using GPy's internal save_model function.
    """
    gp_model.save_model(model_filename)
    rospy.loginfo(f"GP model saved to {model_filename}")



def gp_training_node():
    """
    ROS node for training a Gaussian Process model using GPy.
    """
    # Initialize the ROS node
    rospy.init_node('gp_training_node')

    # Get the data type (wrench or effort) from ROS parameters
    data_type = rospy.get_param('/rosparam/data_type', 'wrench')  # Default to 'effort'

    if data_type == 'wrench':
        training_csv_path = '/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/processed/wrench'
    else:
        training_csv_path = '/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/processed/effort'  # Path to the processed training data CSV
    
    rosbag_name = rospy.get_param('/rosparam/rosbag_name', 'recorded_data.bag')
    rosbag_base_name = os.path.splitext(rosbag_name)[0]
    
    # Get the subsample size from ROS parameters
    subsample_size = rospy.get_param('/rosparam/subsample_size', 5000)  # Default to 5000 if not set
    
    # Depending on the data type, set the train_csv_name appropriately
    if data_type == 'wrench':
        train_csv_param = '_wrench_train_data.csv'
    else:
        train_csv_param = '_effort_train_data.csv'
    
    # Combine the base name with the train_csv_param
    training_csv_name = f"{rosbag_base_name}{train_csv_param}"

    # Combine the path and name using os.path.join (recommended for paths)
    full_train_csv_path = os.path.join(training_csv_path, training_csv_name)
    
    # Model output path, depending on data_type (effort or wrench)
    model_output_path = os.path.join('/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/gp_models', data_type)

    # Combine the output path and model name using os.path.join (recommended for paths)
    full_model_path = os.path.join(model_output_path, f"{rosbag_base_name}_{data_type}_model.pkl")


    # Load the training data
    rospy.loginfo(f"Loading {data_type} training data from {full_train_csv_path} with subsample size {subsample_size}")
    X, Y = load_training_data(full_train_csv_path, subsample_size)

    # Train the GP model
    rospy.loginfo(f"Training Gaussian Process model for {data_type}...")
    gp_model = train_gp_model(X, Y)

    # Save the trained model using GPy's internal method
    save_gp_model(gp_model, full_model_path)


    rospy.loginfo(f"GP training complete. {data_type.capitalize()} model saved at {full_model_path}. Node is shutting down.")

if __name__ == '__main__':
    try:
        gp_training_node()
    except rospy.ROSInterruptException:
        pass
