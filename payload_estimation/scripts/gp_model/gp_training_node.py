#!/usr/bin/env python3

import rospy
import pandas as pd
import GPy
import numpy as np

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
    kernel_type = rospy.get_param('~kernel', 'RBF')

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
    Saves the trained GP model to a file.
    """
    gp_model.save_model(model_filename)
    rospy.loginfo(f"GP model saved to {model_filename}")

def gp_training_node():
    """
    ROS node for training a Gaussian Process model using GPy.
    """
    # Initialize the ROS node
    rospy.init_node('gp_training_node')

    # Parameters for the node (file paths)
    training_csv = rospy.get_param('~training_csv', 
                                   '/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/processed/csv/default_train_data.csv')  # Path to the processed training data CSV
    
    model_output_path = rospy.get_param('~model_output', '/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/gp_models/gp_model.pkl')  # Path to save the trained GP model

    # Load the training data
    rospy.loginfo(f"Loading training data from {training_csv}")
    X, Y = load_training_data(training_csv)

    # Train the GP model
    rospy.loginfo("Training Gaussian Process model...")
    gp_model = train_gp_model(X, Y)

    # Save the trained model to a file
    save_gp_model(gp_model, model_output_path)

    rospy.loginfo("GP training complete. Node is shutting down.")

if __name__ == '__main__':
    try:
        gp_training_node()
    except rospy.ROSInterruptException:
        pass
