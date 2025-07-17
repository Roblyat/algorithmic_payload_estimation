#!/usr/bin/env python3

import rospy
import pandas as pd
import GPy
import numpy as np
import pickle
import os
from sklearn.model_selection import KFold  # Import KFold for cross-validation
from sklearn.metrics import mean_squared_error, mean_absolute_error, r2_score
from scipy.optimize import minimize

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

def infer_latent_space(gplvm_model, X_new, latent_dim):
    """
    Infer the latent space representation for new data points (X_new) using the trained GPLVM model.
    This involves optimizing the latent variables such that the model fits the observed data.
    """
    num_samples = X_new.shape[0]

    # Initialize latent variables for X_new randomly or with some informed guess
    Z_new_init = np.random.randn(num_samples, latent_dim)

    # Define an objective function to optimize the latent variables
    def objective_function(Z_new_flat):
        # Reshape the flat latent variables back to a matrix of shape (num_samples, latent_dim)
        Z_new = Z_new_flat.reshape(num_samples, latent_dim)
        
        # Predict the reconstructed input data from the latent variables
        X_pred, _ = gplvm_model.predict(Z_new)
        
        # Compute the reconstruction error between X_pred and X_new
        return np.sum((X_pred - X_new) ** 2)

    # Optimize the latent variables using a standard optimizer (e.g., L-BFGS-B)
    result = minimize(objective_function, Z_new_init.flatten(), method='L-BFGS-B')
    
    # Reshape the optimized latent variables back to the correct shape
    Z_new_optimized = result.x.reshape(num_samples, latent_dim)
    
    return Z_new_optimized

def train_gplvm_model(X_train, latent_dim, use_sparse):
    """
    Train a Gaussian Process Latent Variable Model (GPLVM) using GPy library.
    - X_train: Features for training (18 features, observed data)
    - latent_dim: The dimensionality of the latent space (set this to 6, as you want a 6D latent space)
    - use_sparse: Boolean to indicate whether to use sparse GPLVM

    Returns the trained GPLVM model.
    """
    # Fix kernel dimension to match the reduced latent space dimension (latent_dim)
    kernel = GPy.kern.RBF(input_dim=latent_dim) + GPy.kern.White(input_dim=latent_dim)

    if use_sparse:

        num_inducing_param = rospy.get_param('/rosparam/num_inducing', 100)  # Default latent dimension is 6
        # Set up sparse GPLVM with a number of inducing points
        num_inducing = min(num_inducing_param, X_train.shape[0])  # Choose an appropriate number of inducing points

        # Sparse GPLVM: input is the observed data (X_train), latent dimension is set by latent_dim
        gplvm_model = GPy.models.SparseGPLVM(X_train, input_dim=latent_dim, num_inducing=num_inducing, kernel=kernel)

        rospy.loginfo(f"Training Sparse GPLVM with {num_inducing} inducing points")
    else:
        # Regular GPLVM: input is the observed data (X_train), latent dimension is set by latent_dim
        gplvm_model = GPy.models.GPLVM(X_train, input_dim=latent_dim, kernel=kernel)
        rospy.loginfo(f"Training regular GPLVM")

    # Optimize the model
    gplvm_model.optimize(messages=True)

    # Print the dimensions to verify consistency
    print(f"Shape of X_train: {X_train.shape}")  # Check shape of the training data
    print(f"Shape of Z_train: {gplvm_model.X.shape}")  # Check shape of the learned latent space

    return gplvm_model


def kfold_train_gplvm_model(X, Y, latent_dims, use_sparse, k=5, r2_threshold=0.50):
    """
    Perform K-Fold Cross-Validation for GPLVM models.
    - X: Features (input joint states)
    - Y: Targets (output force/torque)
    - latent_dims: List of latent dimensions to test
    - use_sparse: Boolean to indicate whether to use sparse GPLVM
    - k: Number of folds for K-Fold CV
    - r2_threshold: The R² threshold to skip the rest of the folds if the first fold is bad
    """
    kf = KFold(n_splits=k, shuffle=True, random_state=42)
    best_gplvm_model = None
    best_gpr_model = None
    best_r2 = -float('inf')  # Initialize best R² to a very low value

    all_mse = []
    all_rmse = []
    all_mae = []
    all_r2 = []

    # Cross-validation loop over latent dimensions
    for latent_dim in latent_dims:
        rospy.loginfo(f"Testing Latent Dimension: {latent_dim}")
        
        fold = 1  # Reset fold count for each latent dimension
        skip_latent_dim = False  # Variable to skip to next latent dim if first fold is bad
        
        for train_index, test_index in kf.split(X):
            X_train, X_test = X[train_index], X[test_index]
            Y_train, Y_test = Y[train_index], Y[test_index]

            rospy.loginfo(f"Training on fold {fold}/{k} with Latent Dimension {latent_dim}...")

            # Train GPLVM model on the training data
            gplvm_model = train_gplvm_model(X_train, latent_dim, use_sparse)

            # Log dimensions after training the GPLVM model
            print(f"Shape of X_train: {X_train.shape}")  # Shape of the input features
            print(f"Shape of Z_train: {gplvm_model.X.shape}")  # Shape of the latent space representation

            # Step 2: Get the latent representation of the training data
            Z_train = gplvm_model.X  # Latent space of the training data

            # Step 2.1: Infer the latent space for test data using the manual optimization approach
            Z_test = infer_latent_space(gplvm_model, X_test, latent_dim)

            # Log dimensions of test latent representation
            print(f"Shape of Z_test: {Z_test.shape}")
            print(f"Shape of Y_train: {Y_train.shape}")  # Shape of the output targets

            rospy.loginfo(f"Shape of Z_train: {Z_train.shape}")
            rospy.loginfo(f"Shape of Z_test: {Z_test.shape}")

            # Step 3: Train a Gaussian Process Regression (GPR) model on the latent space Z_train -> Y_train
            gpr_kernel = GPy.kern.RBF(input_dim=latent_dim) + GPy.kern.White(input_dim=latent_dim)  # Match latent dimension
            gpr_model = GPy.models.GPRegression(Z_train, Y_train, kernel=gpr_kernel)
            gpr_model.optimize(messages=True)

            # Step 4: Predict Y_test from the latent representation Z_test using the GPR model
            Y_pred, _ = gpr_model.predict(Z_test)

            # Step 5: Ensure Y_pred has the correct shape (same as Y_test)
            if Y_pred.shape[1] != Y_test.shape[1]:
                raise ValueError(f"Shape mismatch: Predicted shape {Y_pred.shape} does not match expected shape {Y_test.shape}")

            # Step 6: Calculate the test metrics
            mse = mean_squared_error(Y_test, Y_pred)
            rmse = np.sqrt(mse)  # Root Mean Squared Error
            mae = mean_absolute_error(Y_test, Y_pred)
            r2 = r2_score(Y_test, Y_pred)

            rospy.loginfo(f"Fold {fold} Metrics: MSE={mse}, RMSE={rmse}, MAE={mae}, R^2={r2}")

            # If the R² score for the first fold is too low, skip to the next latent dimension
            if fold == 1 and r2 < r2_threshold:
                rospy.loginfo(f"Skipping Latent Dimension {latent_dim} due to poor R² on the first fold: {r2}")
                skip_latent_dim = True
                break

            # Track all fold metrics
            all_mse.append(mse)
            all_rmse.append(rmse)
            all_mae.append(mae)
            all_r2.append(r2)

            # Update the best models if the current model has a higher R² score
            if r2 > best_r2:
                best_gplvm_model = gplvm_model
                best_gpr_model = gpr_model
                best_r2 = r2

            fold += 1  # Increment fold count for each fold

        # If the first fold is bad, skip this latent dimension
        if skip_latent_dim:
            continue

    # Step 7: Log the average metrics across all folds
    avg_mse = np.mean(all_mse)
    avg_rmse = np.mean(all_rmse)
    avg_mae = np.mean(all_mae)
    avg_r2 = np.mean(all_r2)
    rospy.loginfo(f"Average Metrics Across Folds: MSE={avg_mse}, RMSE={avg_rmse}, MAE={avg_mae}, R^2={avg_r2}")
    rospy.loginfo(f"Best model achieved with R²: {best_r2}")

    return best_gplvm_model, best_gpr_model


def save_gpr_model(gpr_model, model_filename):
    """
    Saves the trained GPR model using pickle.
    """
    try:
        rospy.loginfo(f"Saving GPR model using pickle to {model_filename}")
        with open(model_filename, 'wb') as file:
            pickle.dump(gpr_model, file)
        rospy.loginfo(f"GPR model saved to {model_filename}")
    except Exception as e:
        rospy.logerr(f"Failed to save GPR model to {model_filename}: {str(e)}")


def save_gp_model(gp_model, model_filename):
    """
    Saves the trained GP model using pickle.
    """
    try:
        rospy.loginfo(f"Saving model using pickle to {model_filename}")
        with open(model_filename, 'wb') as file:
            pickle.dump(gp_model, file)
        rospy.loginfo(f"GP model saved to {model_filename}")
    except Exception as e:
        rospy.logerr(f"Failed to save model to {model_filename}: {str(e)}")

def gp_training_node():
    """
    ROS node for training a Gaussian Process Latent Variable Model (GPLVM) using GPy and K-Fold Cross-Validation.
    """
    # Initialize the ROS node
    rospy.init_node('gplvm_kfold_training_node')

    # Get the data type (wrench or effort) from ROS parameters
    data_type = rospy.get_param('/rosparam/data_type', 'wrench')  # Default to 'effort'

    global latent_dims
    # Get list of latent dimensions from ROS parameters
    latent_dims = rospy.get_param('/rosparam/latent_dims', [6, 8, 10, 12])

    # Get latent dimension from ROS parameters
    latent_dim = rospy.get_param('/rosparam/latent_dim', 6)  # Default latent dimension is 6

    # Get the sparse model flag from ROS parameters
    use_sparse = rospy.get_param('/rosparam/use_sparse', False)  # Default to not using sparse model

    if data_type == 'wrench':
        training_csv_path = '/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/processed/wrench'
    else:
        training_csv_path = '/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/processed/effort'  # Path to the processed training data CSV
    
    rosbag_name = rospy.get_param('/rosparam/rosbag_name', 'recorded_data.bag')
    rosbag_base_name = os.path.splitext(rosbag_name)[0]
        
    # Get the subsample size from ROS parameters
    subsample_size = rospy.get_param('/rosparam/subsample_size', 5000)  # Default to 5000 if not set

    use_kfold = rospy.get_param('/rosparam/use_kfold', False)
    
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

    # Initialize the suffix for the model filenames based on the use_sparse and use_kfold flags
    suffix = ""

    if use_kfold and use_sparse:
        suffix = "_k_s"
    elif use_kfold:
        suffix = "_k"
    elif use_sparse:
        suffix = "_s"

    # Combine the output path and model name using os.path.join
    full_gplvm_model_path = os.path.join(model_output_path, f"{rosbag_base_name}_{data_type}_lvm{suffix}_model.pkl")
    full_gpr_model_path = os.path.join(model_output_path, f"{rosbag_base_name}_{data_type}_gpr{suffix}_model.pkl")

    # Load the training data
    rospy.loginfo(f"Loading {data_type} training data from {full_train_csv_path} with subsample size {subsample_size}")
    X, Y = load_training_data(full_train_csv_path, subsample_size)


    if use_kfold:
        # Perform K-Fold cross-validation to train the GPLVM and GPR models
        rospy.loginfo(f"Training GPLVM model for {data_type} with K-Fold CV (Sparse: {use_sparse})...")
        best_gplvm_model, best_gpr_model = kfold_train_gplvm_model(X, Y, latent_dims, use_sparse, k=5, r2_threshold=0.30)
    else:
        # Train the GPLVM model (non-KFold case, simplified for demonstration)
        rospy.loginfo(f"Training GPLVM model for {data_type} (Sparse: {use_sparse})...")
        best_gplvm_model = train_gplvm_model(X, latent_dim, use_sparse)
        # Train a GPR model on the entire training data using inferred latent space
        Z_train = best_gplvm_model.X
        gpr_kernel = GPy.kern.RBF(input_dim=latent_dim) + GPy.kern.White(input_dim=latent_dim)
        best_gpr_model = GPy.models.GPRegression(Z_train, Y, kernel=gpr_kernel)
        best_gpr_model.optimize(messages=True)

    # Save the trained GPLVM model
    save_gp_model(best_gplvm_model, full_gplvm_model_path)

    # Save the trained GPR model
    rospy.loginfo(f"GPLVM training complete. {data_type.capitalize()} GPLVM model saved at {full_gplvm_model_path}. Saving GPR model now.")
    save_gpr_model(best_gpr_model, full_gpr_model_path)

    rospy.loginfo(f"GPR model saved at {full_gpr_model_path}. Node is shutting down.")


if __name__ == '__main__':
    try:
        gp_training_node()
    except rospy.ROSInterruptException:
        pass