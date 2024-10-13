#!/usr/bin/env python3

import rospy
import numpy as np
import pickle
import os
from sensor_msgs.msg import JointState  # For /joint_states topic
from payload_estimation.msg import PredictedEffort  # Import the custom message
import pandas as pd
import GPy

# UR5 joint names to filter from the joint_states message
UR5_JOINTS = ['ur5_elbow_joint', 'ur5_shoulder_lift_joint', 'ur5_shoulder_pan_joint', 
              'ur5_wrist_1_joint', 'ur5_wrist_2_joint', 'ur5_wrist_3_joint']

def load_gp_model(model_filename):
    """
    Load the trained GP model using GPy's internal load_model function.
    """
    rospy.loginfo(f"Loading GP model from {model_filename}")
    gp_model = GPy.core.GP.load_model(model_filename)
    rospy.loginfo("GP model loaded successfully.")
    return gp_model



def load_scaler(scaler_filename):
    """
    Load the StandardScaler and its feature names used during training from a file.
    """
    rospy.loginfo(f"Loading scaler and feature names from {scaler_filename}")
    with open(scaler_filename, 'rb') as f:
        scaler_data = pickle.load(f)
    
    scaler = scaler_data['scaler']
    feature_names = scaler_data['columns']  # Extract the feature names
    rospy.loginfo("Scaler and feature names loaded successfully.")
    return scaler, feature_names


def preprocess_input(joint_state_msg, scaler, feature_names):
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
        acceleration = velocity / 0.025  # Compute acceleration with delta time 40 Hz (1/40 s)
        
        # Print/log the joint states for each joint
        rospy.loginfo(f"Joint {joint_name}: Position={position}, Velocity={velocity}, Acceleration={acceleration}")
        
        # Append the values to the input vector
        input_vector.append(position)
        input_vector.append(velocity)
        input_vector.append(acceleration)

    # Print/log the complete input vector before scaling
    rospy.loginfo(f"Input vector before scaling: {input_vector}")

    # Print the scaler's mean and scale values
    rospy.loginfo(f"Scaler mean: {scaler.mean_}")
    rospy.loginfo(f"Scaler scale (std): {scaler.scale_}")

    # Convert input_vector into a DataFrame using the feature names loaded from training
    input_df = pd.DataFrame([input_vector], columns=feature_names)

    # Standardize the input using the loaded scaler
    input_vector_scaled = scaler.transform(input_df)

    # Print/log the input vector after scaling
    rospy.loginfo(f"Input vector after scaling: {input_vector_scaled}")

    return input_vector_scaled




def predict_with_gp(gp_model, input_vector):
    """
    Make predictions using the trained GP model on the real-time data.
    - input_vector: The feature inputs from the current /joint_states data.
    
    Returns the predicted values and the variances.
    """
    Y_pred, Y_var = gp_model.predict(input_vector)
    return Y_pred, Y_var

def joint_state_callback(joint_state_msg):
    """
    Callback for the /joint_states topic. This function extracts joint data,
    preprocesses it, and uses the GP model to make predictions.
    """
    # Preprocess the incoming joint_states message to match the model input
    input_vector = preprocess_input(joint_state_msg, scaler, feature_names)

    # Make a prediction using the GP model
    Y_pred, Y_var = predict_with_gp(gp_model, input_vector)

    # Log or publish the predictions (e.g., efforts or wrench data)
    rospy.loginfo(f"Predicted: {Y_pred}, Variance: {Y_var}")

    # Prepare the output message (PredictedEffort)
    output_msg = PredictedEffort()

    # Fill the output vector (combining joint positions, velocities, and predicted efforts)
    output_vector = []
    for i, joint_name in enumerate(UR5_JOINTS):
        index = joint_state_msg.name.index(joint_name)
        output_vector.append(joint_state_msg.position[index])   # Joint position
        output_vector.append(joint_state_msg.velocity[index])   # Joint velocity
        output_vector.append(Y_pred[0][i])                      # Predicted effort for this joint

    # Fill in the PredictedEffort message fields
    output_msg.wrench_input = output_vector  # Predicted targets of effort_gp_model
    output_msg.variance = Y_var[0].tolist()  # Convert numpy array to list
    output_msg.timestamp = joint_state_msg.header.stamp  # Use the same timestamp as joint states

    # Log the output vector
    rospy.loginfo(f"Output: {output_msg}")

    # Publish the predictions
    prediction_pub.publish(output_msg)

def gp_live_prediction_node():
    """
    ROS node for live prediction of efforts or wrench using Gaussian Process models.
    """
    rospy.init_node('gp_live_prediction_node')

    # Load parameters: rosbag name and data type (effort or wrench)
    rosbag_name = rospy.get_param('/rosparam/rosbag_name', 'recorded_data.bag')
    data_type = rospy.get_param('/rosparam/data_type', 'effort')  # Default to 'effort'

    rosbag_base_name = os.path.splitext(rosbag_name)[0]

    # Load the correct GP model based on rosbag name and data type
    model_path = os.path.join('/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/gp_models', data_type)
    model_filename = os.path.join(model_path, f"{rosbag_base_name}_{data_type}_k_model.pkl.zip")
    scaler_path = os.path.join('/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/gp_models/scalers', data_type)
    scaler_filename = os.path.join(scaler_path, f"{rosbag_base_name}_{data_type}_scaler.pkl")  # Path to the saved scaler

    global gp_model
    gp_model = load_gp_model(model_filename)

    global scaler, feature_names
    scaler, feature_names = load_scaler(scaler_filename)

    # Subscribe to the /joint_states topic to get live data
    rospy.Subscriber('/joint_states', JointState, joint_state_callback)

    # Publisher for predictions
    global prediction_pub
    prediction_pub = rospy.Publisher('/predicted_effort', PredictedEffort, queue_size=10)

    rospy.loginfo("GP live prediction node is running...")
    rospy.spin()


if __name__ == '__main__':
    try:
        gp_live_prediction_node()
    except rospy.ROSInterruptException:
        pass