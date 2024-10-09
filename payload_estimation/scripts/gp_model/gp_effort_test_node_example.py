#!/usr/bin/env python3

import rospy
import pandas as pd
import numpy as np
import pickle

def load_test_data(test_csv):
    """
    Loads the test data from a CSV file.
    Assumes the last 6 columns are the target (Efforts), and the rest are the features.
    """
    df_test = pd.read_csv(test_csv)

    # Split features and targets
    X_test = df_test.iloc[:, :-6].values  # Features: all columns except the last 6
    Y_test = df_test.iloc[:, -6:].values  # Targets: last 6 columns (Efforts)
    
    return X_test, Y_test

def load_gp_model(model_filename):
    """
    Load the trained GP model from file using pickle.
    """
    rospy.loginfo(f"Loading GP model from {model_filename}")
    with open(model_filename, 'rb') as f:
        gp_model = pickle.load(f)  # Load the model using pickle
    rospy.loginfo("GP model loaded successfully.")
    return gp_model

def predict_with_gp(gp_model, X_test):
    """
    Make predictions using the trained GP model on the test data.
    - X_test: The feature inputs from the test dataset.
    
    Returns the predicted effort values and the variances.
    """
    Y_pred, Y_var = gp_model.predict(X_test)

    # Debug: print or log the shapes
    rospy.loginfo(f"Y_pred shape: {Y_pred.shape}, Y_var shape: {Y_var.shape}")
    
    return Y_pred, Y_var


def gp_test_node():
    """
    ROS node for testing the Gaussian Process model using a test dataset.
    """
    # Initialize the ROS node
    rospy.init_node('gp_test_node')

    # Parameters (file paths)
    test_csv = rospy.get_param('~test_csv', '/path/to/test_data.csv')  # Path to the test dataset
    model_filename = rospy.get_param('~model_filename', '/path/to/gp_model.pkl')  # Path to the trained GP model
    results_csv = rospy.get_param('~results_csv', '/path/to/results.csv')  # Path to save the results

    # Load the test data
    rospy.loginfo(f"Loading test data from {test_csv}")
    X_test, Y_test = load_test_data(test_csv)

    # Load the GP model
    gp_model = load_gp_model(model_filename)

    # Make predictions on the test data
    rospy.loginfo("Making predictions on the test dataset...")
    Y_pred, Y_var = predict_with_gp(gp_model, X_test)

    # Save the actual vs predicted results to a CSV file
    rospy.loginfo(f"Saving predictions and actual values to {results_csv}")
    
    # Concatenate the test data, predicted values, and variance (one variance column per effort)
    df_results = pd.DataFrame(np.hstack((Y_test, Y_pred, Y_var)),
                            columns=['Actual Effort Joint 1', 'Actual Effort Joint 2', 'Actual Effort Joint 3',
                                     'Actual Effort Joint 4', 'Actual Effort Joint 5', 'Actual Effort Joint 6',
                                     'Predicted Effort Joint 1', 'Predicted Effort Joint 2', 'Predicted Effort Joint 3',
                                     'Predicted Effort Joint 4', 'Predicted Effort Joint 5', 'Predicted Effort Joint 6',
                                    'Variance'])

    df_results.to_csv(results_csv, index=False)
    rospy.loginfo(f"Results saved to {results_csv}")

if __name__ == '__main__':
    try:
        gp_test_node()
    except rospy.ROSInterruptException:
        pass
