#!/usr/bin/env python3

import rospy
import numpy as np
import pickle
import os
from sensor_msgs.msg import JointState  # For /joint_states topic
from payload_estimation.msg import PredictedEffort  # Import the custom message
from std_msgs.msg import Float64MultiArray
import pandas as pd
import GPy
from scipy.optimize import minimize

# UR5 joint names to filter from the joint_states message
UR5_JOINTS = ['ur5_elbow_joint', 'ur5_shoulder_lift_joint', 'ur5_shoulder_pan_joint', 
              'ur5_wrist_1_joint', 'ur5_wrist_2_joint', 'ur5_wrist_3_joint']

def load_models(gplvm_filename, gpr_filename):
    """
    Load the trained GPLVM and GPR models using pickle.
    """
    try:
        rospy.loginfo(f"Loading GPLVM model from {gplvm_filename}")
        with open(gplvm_filename, 'rb') as file:
            gplvm_model = pickle.load(file)
        rospy.loginfo(f"GPLVM model loaded successfully.")
        
        rospy.loginfo(f"Loading GPR model from {gpr_filename}")
        with open(gpr_filename, 'rb') as file:
            gpr_model = pickle.load(file)
        rospy.loginfo(f"GPR model loaded successfully.")

        return gplvm_model, gpr_model
    except Exception as e:
        rospy.logerr(f"Failed to load models: {str(e)}")
        return None, None


def load_X_scaler(X_scaler_filename):
    """
    Load the StandardScaler and its feature names used during training from a file.
    """
    rospy.loginfo(f"Loading scaler and feature names from {X_scaler_filename}")
    with open(X_scaler_filename, 'rb') as f:
        scaler_data = pickle.load(f)
    
    scaler = scaler_data['scaler']
    feature_names = scaler_data['columns']  # Extract the feature names
    rospy.loginfo("Scaler and feature names loaded successfully.")
    return scaler, feature_names

def load_Y_scaler(Y_scaler_filename):
    """
    Load the StandardScaler and its target names used during training from a file.
    """
    rospy.loginfo(f"Loading scaler and target names from {Y_scaler_filename}")
    with open(Y_scaler_filename, 'rb') as f:
        scaler_data = pickle.load(f)
    
    scaler = scaler_data['scaler']
    feature_names = scaler_data['columns']  # Extract the target names
    rospy.loginfo("Scaler and targets names loaded successfully.")
    return scaler, feature_names

def preprocess_input(joint_state_msg, X_scaler, X_feature_names):
    """
    Extract and format the input features from the /joint_states topic message.
    This will create an input vector in the format that matches the GP model's training data.
    """
    positions = []
    velocities = []

    # Extract only the UR5 joints from the joint_state message
    for joint_name in UR5_JOINTS:
        if joint_name in joint_state_msg.name:
            index = joint_state_msg.name.index(joint_name)
            positions.append(joint_state_msg.position[index])
            velocities.append(joint_state_msg.velocity[index])

    # Build the input vector based on positions and velocities
    input_vector = []
    rospy.loginfo("Joint States for UR5 Joints:")
    for i, joint_name in enumerate(UR5_JOINTS):  # Assuming 6 joints for the UR5
        position = positions[i]
        velocity = velocities[i]
        acceleration = (velocity - velocities[i-1]) / 0.025  # Compute acceleration with delta time 40 Hz (1/40 s)
        
        rospy.loginfo(f"Joint {joint_name}: Position={position}, Velocity={velocity}, Acceleration={acceleration}")
        
        input_vector.append(position)
        input_vector.append(velocity)
        input_vector.append(acceleration)

    input_df = pd.DataFrame([input_vector], columns=X_feature_names)
    input_vector_scaled = X_scaler.transform(input_df)

    # Print the shape of the input vector for debugging
    print(f"Shape of input_vector: {input_vector_scaled.shape}")

    return input_vector_scaled

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

def predict_with_models(gplvm_model, gpr_model, input_vector):
    """
    Make predictions using the trained GPLVM and GPR models on the real-time data.
    """
    # Ensure input vector is 2D (even for a single sample)
    input_vector = np.atleast_2d(input_vector)  # Ensures shape (1, 18) or (1, <number of features>)

    # Infer the latent space representation for the input vector using the GPLVM model
    latent_dim = gplvm_model.input_dim  # Assuming the GPLVM model has this attribute
    Z_input = infer_latent_space(gplvm_model, input_vector, latent_dim)

    # Predict using the GPR model
    Y_pred, Y_var = gpr_model.predict(Z_input)
    
    # Print the shape of Y_pred for debugging
    print(f"Shape of Y_pred: {Y_pred.shape}")

    return Y_pred, Y_var


def joint_state_callback(joint_state_msg):
    """
    Callback for the /joint_states topic. This function extracts joint data,
    preprocesses it, and uses the models (GPLVM and GPR) to make predictions.
    """
    input_vector = preprocess_input(joint_state_msg, X_scaler, X_feature_names)
    Y_pred, Y_var = predict_with_models(gplvm_model, gpr_model, input_vector)

    # Ensure Y_pred has the correct shape (should be (1, 6) if predicting 6 target values)
    try:
        if Y_pred.shape[1] != 6:
            raise ValueError(f"Expected Y_pred to have 6 columns, but got {Y_pred.shape[1]} columns")

        # Adjust the inverse transformation to only take the first 6 columns if necessary
        Y_pred_original = Y_scaler.inverse_transform(Y_pred[:, :6])
    except ValueError as e:
        rospy.logerr(f"Error reshaping Y_pred for inverse transform: {str(e)}")
        return

    rospy.loginfo(f"Predicted: {Y_pred_original}, Variance: {Y_var}")

    output_msg = PredictedEffort()
    output_vector = []
    for i, joint_name in enumerate(UR5_JOINTS):
        index = joint_state_msg.name.index(joint_name)
        output_vector.append(joint_state_msg.position[index])
        output_vector.append(joint_state_msg.velocity[index])
        output_vector.append(Y_pred_original[0][i])

    output_msg.wrench_input = output_vector
    output_msg.variance = Y_var[0].tolist()
    output_msg.timestamp = joint_state_msg.header.stamp

    rospy.loginfo(f"Output: {output_msg}")
    wrench_input = Float64MultiArray()
    wrench_input.data = Y_pred_original.flatten().tolist()

    prediction_pub.publish(wrench_input)


def gp_live_prediction_node():
    """
    ROS node for live prediction of efforts or wrench using Gaussian Process models.
    """
    rospy.init_node('gp_effort_prediction_node')
    rosbag_name = rospy.get_param('/rosparam/rosbag_name', 'recorded_data.bag')
    data_type = rospy.get_param('/rosparam/data_type', 'effort')

    if data_type != 'effort':
        rospy.logerr(f"Data type is '{data_type}'. This script only supports 'effort'. Shutting down.")
        rospy.signal_shutdown("Unsupported data type")
        return

    rosbag_base_name = os.path.splitext(rosbag_name)[0]

    global gplvm_model, gpr_model
    gplvm_model, gpr_model = load_models(
        gplvm_filename=os.path.join('/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/gp_models/effort', f"{rosbag_base_name}_{data_type}_lvm_k_s_model.pkl"),
        gpr_filename=os.path.join('/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/gp_models/effort', f"{rosbag_base_name}_{data_type}_gpr_k_s_model.pkl")
    )

    global X_scaler, X_feature_names
    X_scaler, X_feature_names = load_X_scaler(
        os.path.join('/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/gp_models/scalers/effort', f"{rosbag_base_name}_{data_type}_feature_scaler.pkl")
    )

    global Y_scaler, Y_feature_names
    Y_scaler, Y_feature_names = load_Y_scaler(
        os.path.join('/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/gp_models/scalers/effort', f"{rosbag_base_name}_{data_type}_target_scaler.pkl")
    )

    rospy.Subscriber('/joint_states', JointState, joint_state_callback)

    global prediction_pub
    prediction_pub = rospy.Publisher('/predicted_effort', Float64MultiArray, queue_size=10)

    rospy.loginfo("GP/GPLVM live prediction node is running...")
    rospy.spin()


if __name__ == '__main__':
    try:
        gp_live_prediction_node()
    except rospy.ROSInterruptException:
        pass
