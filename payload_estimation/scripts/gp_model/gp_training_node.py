#!/usr/bin/env python3

import rospy
import pandas as pd
import GPy
import numpy as np
import pickle  # Import pickle for saving/loading models
import os  # For joining paths

def load_training_data(training_csv):
    """
    Loads the training data from a CSV file.
    Assumes the last 6 columns are the target (Force/Torque), and the rest are the features.
    """
    df_training = pd.read_csv(training_csv)
    
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

def save_gp_model_pickle(gp_model, model_filename):
    """
    Saves the trained GP model to a file using pickle.
    """
    with open(model_filename, 'wb') as f:
        pickle.dump(gp_model, f)  # Save the model using pickle
    rospy.loginfo(f"GP model saved to {model_filename}")

def load_gp_model_pickle(model_filename):
    """
    Loads the trained GP model from a file using pickle.
    """
    with open(model_filename, 'rb') as f:
        gp_model = pickle.load(f)  # Load the model using pickle
    return gp_model

def gp_training_node():
    """
    ROS node for training a Gaussian Process model using GPy.
    """
    # Initialize the ROS node
    rospy.init_node('gp_training_node')

    # Parameters for the node (file paths)
    training_csv_path = rospy.get_param('/rosparam/train_csv_path', 
                                   '/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/processed')  # Path to the processed training data CSV
    training_csv_name = rospy.get_param('/rosparam/train_csv_name', 'default_train_data.csv')

    # Combine the path and name using os.path.join (recommended for paths)
    full_train_csv_path = os.path.join(training_csv_path, training_csv_name)
    
    model_output_path = rospy.get_param('rosparam/model_output', '/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/gp_models')
    model_name_param = rospy.get_param('/rosparam/model_name', 'gp_model.pkl')

    # Combine the output path and model name using os.path.join (recommended for paths)
    full_model_path = os.path.join(model_output_path, model_name_param)

    # Load the training data
    rospy.loginfo(f"Loading training data from {full_train_csv_path}")
    X, Y = load_training_data(full_train_csv_path)

    # Train the GP model
    rospy.loginfo("Training Gaussian Process model...")
    gp_model = train_gp_model(X, Y)

    # Save the trained model using pickle
    save_gp_model_pickle(gp_model, full_model_path)

    rospy.loginfo(f"GP training complete. Model saved at {full_model_path}. Node is shutting down.")

if __name__ == '__main__':
    try:
        gp_training_node()
    except rospy.ROSInterruptException:
        pass