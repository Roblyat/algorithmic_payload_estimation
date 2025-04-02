#!/usr/bin/env python3

import rospy
import pandas as pd
import GPy
import numpy as np
import pickle
import os
from sklearn.model_selection import KFold  # Import KFold for cross-validation
from sklearn.metrics import mean_squared_error, mean_absolute_error, r2_score

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

def train_gp_model(X_train, Y_train, X_test, Y_test, kernel, use_sparse, model_filename):
    """
    Train a Gaussian Process (GP) model using GPy library.
    - X_train: Features for training
    - Y_train: Targets for training
    - X_test: Features for testing/validation
    - Y_test: Targets for testing/validation
    - kernel: Kernel for the GP model.
    - use_sparse: Boolean to determine if sparse GP should be used.
    - model_filename: Path to save model info as a txt file.
    
    Returns the trained GP model and the test metrics (MSE, RMSE, MAE, R^2).
    """
    # Create the GP regression model
    if use_sparse:
        inducing_points = rospy.get_param('/rosparam/num_inducing', 2500)   #X_train[np.random.choice(X_train.shape[0], min(500, X_train.shape[0]), replace=False)]
        gp_model = GPy.models.SparseGPRegression(X_train, Y_train, kernel, num_inducing=inducing_points)    #Z=inducing_points)
        rospy.loginfo("Using sparse GP model...")
    else:
        gp_model = GPy.models.GPRegression(X_train, Y_train, kernel)
        rospy.loginfo("Using full GP model...")

    rospy.loginfo("Optimizing the GP model...")
    gp_model.optimize(messages=True)

    # gp_model.optimize_restarts(num_restarts=1, verbose=True, max_f_eval=500, max_iters=500)

    # Capture model and kernel print output
    model_info = f"Optimized GP model structure:\n{gp_model}\n\nKernel structure:\n{gp_model.kern}"

    # Save the model info into a text file
    info_filename = model_filename.replace('.pkl', '_info.txt')
    with open(info_filename, 'w') as f:
        f.write(model_info)

    rospy.loginfo(f"Model information saved to {info_filename}")

    # Predict the output for the test set
    Y_pred, _ = gp_model.predict(X_test)

    # Calculate the test metrics
    mse = mean_squared_error(Y_test, Y_pred)
    rmse = np.sqrt(mse)
    mae = mean_absolute_error(Y_test, Y_pred)
    r2 = r2_score(Y_test, Y_pred)

    # Log the metrics
    rospy.loginfo(f"Test MSE: {mse}, Test RMSE: {rmse}, Test MAE: {mae}, Test R^2: {r2}")

    return gp_model, mse, rmse, mae, r2


def kfold_train_gp_model(X, Y, kernel, use_sparse, k=5):
    """
    Perform K-Fold Cross-Validation for Gaussian Process models.
    - X: Features (input joint states)
    - Y: Targets (output force/torque)
    - kernel: The GP kernel to use
    - use_sparse: Boolean to determine if sparse GP should be used
    - k: Number of folds for K-Fold CV
    
    Returns the best GP model based on the highest R² score.
    """
    kf = KFold(n_splits=k, shuffle=True, random_state=42)
    fold = 1
    best_model = None
    best_r2 = -float('inf')  # Initialize best R² to a very low value

    all_mse = []
    all_rmse = []
    all_mae = []
    all_r2 = []

    # Cross-validation loop
    for train_index, test_index in kf.split(X):
        X_train, X_test = X[train_index], X[test_index]
        Y_train, Y_test = Y[train_index], Y[test_index]

        rospy.loginfo(f"Training on fold {fold}/{k}...")
        
        # Train GP model on this fold
        gp_model, mse, rmse, mae, r2 = train_gp_model(X_train, Y_train, X_test, Y_test, kernel, use_sparse, model_filename=f"temp_fold_{fold}_model.txt")

        rospy.loginfo(f"Fold {fold} Metrics: MSE={mse}, RMSE={rmse}, MAE={mae}, R^2={r2}")

        # Track all fold metrics
        all_mse.append(mse)
        all_rmse.append(rmse)
        all_mae.append(mae)
        all_r2.append(r2)

        # Update the best model if current model has a higher R² score
        if r2 > best_r2:
            best_model = gp_model
            best_r2 = r2

        fold += 1

    # Log the average metrics across all folds
    avg_mse = np.mean(all_mse)
    avg_rmse = np.mean(all_rmse)
    avg_mae = np.mean(all_mae)
    avg_r2 = np.mean(all_r2)
    rospy.loginfo(f"Average Metrics Across Folds: MSE={avg_mse}, RMSE={avg_rmse}, MAE={avg_mae}, R^2={avg_r2}")
    rospy.loginfo(f"Best model achieved with R²: {best_r2}")

    return best_model


def save_gp_model(gp_model, model_filename):
    """
    Saves the trained GP model using pickle.
    """
    try:
        rospy.loginfo(f"Saving model using pickle to {model_filename}")

        # Save the model with pickle
        with open(model_filename, 'wb') as file:
            pickle.dump(gp_model, file)
        rospy.loginfo(f"GP model saved to {model_filename}")

    except Exception as e:
        rospy.logerr(f"Failed to save model to {model_filename}: {str(e)}")


def gp_training_node():
    """
    ROS node for training a Gaussian Process model using GPy and K-Fold Cross-Validation.
    """
    # Initialize the ROS node
    rospy.init_node('gp_kfold_training_node')

    # Get the data type (wrench or effort) from ROS parameters
    data_type = rospy.get_param('/rosparam/data_type', 'wrench')  # Default to 'wrench'

    if data_type == 'wrench':
        training_csv_path = '/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/processed/wrench'
    else:
        training_csv_path = '/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/processed/effort'
    
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

    # Check if sparse GP models are being used
    use_sparse = rospy.get_param('/rosparam/use_sparse', False)
    suffix = "_k_s" if use_sparse else "_k"

    # Combine the output path and model name using os.path.join (recommended for paths)
    full_model_path = os.path.join(model_output_path, f"{rosbag_base_name}_{data_type}{suffix}_model.pkl")

    # Load the training data
    rospy.loginfo(f"Loading {data_type} training data from {full_train_csv_path} with subsample size {subsample_size}")
    X, Y = load_training_data(full_train_csv_path, subsample_size)

    # Define the kernel (RBF by default or other types)
    kernel_type = rospy.get_param('/rosparam/kernel', 'RBF')
    if kernel_type == 'RBF':
        kernel = GPy.kern.RBF(input_dim=X.shape[1], variance=1., lengthscale=1.)
    elif kernel_type == 'Matern52':
        kernel = GPy.kern.Matern52(input_dim=X.shape[1], variance=1., lengthscale=1.)
    elif kernel_type == 'Linear':
        kernel = GPy.kern.Linear(input_dim=X.shape[1])
    elif kernel_type == 'RBF_White':
        # Create a combination of RBF and White noise kernels
        rbf_kernel = GPy.kern.RBF(input_dim=X.shape[1], variance=1., lengthscale=1.)
        white_kernel = GPy.kern.White(input_dim=X.shape[1], variance=1e-5)  # A small default variance for noise
        kernel = rbf_kernel + white_kernel
    else:
        rospy.logwarn(f"Unknown kernel type '{kernel_type}', defaulting to RBF.")
        kernel = GPy.kern.RBF(input_dim=X.shape[1], variance=1., lengthscale=1.)

    # Perform K-Fold cross-validation to train the GP model
    rospy.loginfo(f"Training Gaussian Process model for {data_type} with K-Fold CV (Sparse: {use_sparse})...")
    best_gp_model = kfold_train_gp_model(X, Y, kernel, use_sparse, k=5)

    # Save the best trained model and K-Fold metrics
    save_gp_model(best_gp_model, full_model_path)


    rospy.loginfo(f"GP training complete. {data_type.capitalize()} model saved at {full_model_path}. Node is shutting down.") 

if __name__ == '__main__':
    try:
        gp_training_node()
    except rospy.ROSInterruptException:
        pass