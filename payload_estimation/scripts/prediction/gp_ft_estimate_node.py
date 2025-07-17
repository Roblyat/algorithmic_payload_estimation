#!/usr/bin/env python3

import rospy
import numpy as np
import pickle
import os
import pandas as pd
import GPy
from sensor_msgs.msg import JointState  # For /joint_states topic
from std_msgs.msg import Float64MultiArray  # For /predicted_effort topic
from geometry_msgs.msg import WrenchStamped  # Wrench message for the predicted forces/torques

# UR5 joint names to filter from the joint_states message
UR5_JOINTS = ['ur5_elbow_joint', 'ur5_shoulder_lift_joint', 'ur5_shoulder_pan_joint', 
              'ur5_wrist_1_joint', 'ur5_wrist_2_joint', 'ur5_wrist_3_joint']

def load_gp_model(model_filename):
    """
    Load the trained GP model using pickle.
    """
    try:
        rospy.loginfo(f"Loading GP model from {model_filename}")
        with open(model_filename, 'rb') as file:
            gp_model = pickle.load(file)
        rospy.loginfo("GP model loaded successfully.")
        return gp_model
    except Exception as e:
        rospy.logerr(f"Failed to load model from {model_filename}: {str(e)}")
        return None

def load_X_scaler(X_scaler_filename):
    rospy.loginfo(f"Loading scaler and feature names from {X_scaler_filename}")
    with open(X_scaler_filename, 'rb') as f:
        scaler_data = pickle.load(f)
    scaler = scaler_data['scaler']
    feature_names = scaler_data['columns']  # Extract the feature names
    rospy.loginfo("Scaler and feature names loaded successfully.")
    return scaler, feature_names

def load_Y_scaler(Y_scaler_filename):
    rospy.loginfo(f"Loading scaler and target names from {Y_scaler_filename}")
    with open(Y_scaler_filename, 'rb') as f:
        scaler_data = pickle.load(f)
    scaler = scaler_data['scaler']
    feature_names = scaler_data['columns']  # Extract the target names
    rospy.loginfo("Scaler and targets names loaded successfully.")
    return scaler, feature_names

def preprocess_joint_states(joint_state_msg):
    """
    Extract positions and velocities from joint states.
    """
    positions = []
    velocities = []
    
    # Extract only the UR5 joints from the joint_state message
    for joint_name in UR5_JOINTS:
        if joint_name in joint_state_msg.name:
            index = joint_state_msg.name.index(joint_name)
            positions.append(joint_state_msg.position[index])
            velocities.append(joint_state_msg.velocity[index])
    
    return positions, velocities

def preprocess_effort(positions, velocities):
    """
    Process joint states callback for the effort GP model.
    """

    # Build the input vector based on positions and velocities
    input_vector = []
    rospy.loginfo("Joint States for UR5 Joints:")
    for i, joint_name in enumerate(UR5_JOINTS):  # Assuming 6 joints for the UR5
        position = positions[i]
        velocity = velocities[i]
        acceleration = (velocity - velocities[i-1]) / 0.025  # Compute acceleration with delta time 40 Hz (1/40 s)
        
        # Print/log the joint states for each joint
        rospy.loginfo(f"Joint {joint_name}: Position={position}, Velocity={velocity}, Acceleration={acceleration}")
        
        # Append the values to the input vector
        input_vector.append(position)
        input_vector.append(velocity)
        input_vector.append(acceleration)

    # Print/log the complete input vector before scaling
    rospy.loginfo(f"Input vector before scaling: {input_vector}")

    # Print the scaler's mean and scale values
    rospy.loginfo(f"Scaler mean: {effort_X_scaler.mean_}")
    rospy.loginfo(f"Scaler scale (std): {effort_X_scaler.scale_}")

    # Convert input_vector into a DataFrame using the feature names loaded from training
    input_df = pd.DataFrame([input_vector], columns=effort_X_feature_names)

    # Standardize the input using the loaded scaler
    input_vector_scaled = effort_X_scaler.transform(input_df)

    # Print/log the input vector after scaling
    rospy.loginfo(f"Input vector after scaling: {input_vector_scaled}")

    return input_vector_scaled


def predict_with_gp(gp_model, input_vector):
    Y_pred, Y_var = gp_model.predict(input_vector)
    return Y_pred, Y_var

def publish_wrench_prediction(Y_pred_original):
    """
    Publish the predicted wrench (force/torque) using a WrenchStamped message.
    """
    output_msg = WrenchStamped()
    output_msg.header.stamp = rospy.Time.now()
    
    # Predicted forces (X, Y, Z)
    output_msg.wrench.force.x = Y_pred_original[0][0]  # Predicted Force X
    output_msg.wrench.force.y = Y_pred_original[0][1]  # Predicted Force Y
    output_msg.wrench.force.z = Y_pred_original[0][2]  # Predicted Force Z
    
    # Predicted torques (X, Y, Z)
    output_msg.wrench.torque.x = Y_pred_original[0][3]  # Predicted Torque X
    output_msg.wrench.torque.y = Y_pred_original[0][4]  # Predicted Torque Y
    output_msg.wrench.torque.z = Y_pred_original[0][5]  # Predicted Torque Z

    # Log the wrench prediction to ROS for debugging
    rospy.loginfo(f"Publishing Wrench Prediction:")
    rospy.loginfo(f"Force - X: {output_msg.wrench.force.x}, Y: {output_msg.wrench.force.y}, Z: {output_msg.wrench.force.z}")
    rospy.loginfo(f"Torque - X: {output_msg.wrench.torque.x}, Y: {output_msg.wrench.torque.y}, Z: {output_msg.wrench.torque.z}")

    # Publish the message
    wrench_pub.publish(output_msg)


def joint_state_callback(joint_state_msg):
    """
    Callback for /joint_states.
    Store joint state information for use in wrench prediction.
    """
    joint_positions, joint_velocities = preprocess_joint_states(joint_state_msg)

    # Preprocess and scale the effort input vector
    effort_input_vector = preprocess_effort(joint_positions, joint_velocities)
    
    # Predict effort using GP model and scale it back using inverse_transform
    effort_prediction, _ = predict_with_gp(effort_gp_model, effort_input_vector)  # Get only Y_pred from the tuple
    predicted_effort_original = effort_Y_scaler.inverse_transform(effort_prediction)  # Inverse scale the effort prediction

    # Flatten the predicted effort to feed it into the wrench model
    predicted_effort = predicted_effort_original.flatten().tolist()

    # Create a Float64MultiArray message for publishing
    effort_msg = Float64MultiArray()
    effort_msg.data = predicted_effort

    # Publish the predicted effort to the /predicted_effort topic
    predicted_effort_pub.publish(effort_msg)
    rospy.loginfo(f"Published predicted effort: {predicted_effort}")

    # Preprocess wrench inputs using the original (inverse-scaled) effort prediction
    wrench_input_vector = preprocess_wrench(predicted_effort, joint_positions, joint_velocities)
    
    # Predict the wrench using the GP model and scale it back using inverse_transform
    wrench_prediction, _ = predict_with_gp(wrench_gp_model, wrench_input_vector)
    wrench_prediction_original = wrench_Y_scaler.inverse_transform(wrench_prediction)  # Inverse scale the wrench prediction

    # Publish the final wrench prediction after inverse scaling
    publish_wrench_prediction(wrench_prediction_original)


def preprocess_wrench(predicted_effort, joint_positions, joint_velocities):
    """
    Preprocess the predicted effort and joint states for the wrench GP model.
    """
   
    # Log the shape or length of the predicted efforts
    rospy.loginfo(f"Effort prediction length: {len(predicted_effort)}")
    
    wrench_input = []
    for i, joint_name in enumerate(UR5_JOINTS):
        # Ensure the effort_prediction has the correct number of elements
        if len(predicted_effort) <= i:
            rospy.logerr(f"Effort prediction does not have enough elements for joint {i}: {predicted_effort}")
            continue
        
        # Use joint_positions and joint_velocities passed to the function
        wrench_input.append(joint_positions[i])   # Joint position
        wrench_input.append(joint_velocities[i])  # Joint velocity
        wrench_input.append(predicted_effort[i])  # Predicted effort for this joint

    # Log the output vector
    rospy.loginfo(f"Wrench input vector: {wrench_input}")

    # Print the scaler's mean and scale values
    rospy.loginfo(f"Scaler mean: {wrench_X_scaler.mean_}")
    rospy.loginfo(f"Scaler scale (std): {wrench_X_scaler.scale_}")

    # Convert input_vector into a DataFrame using the feature names loaded from training
    input_df = pd.DataFrame([wrench_input], columns=wrench_X_feature_names)

    # Standardize the input using the loaded scaler
    wrench_input_vector_scaled = wrench_X_scaler.transform(input_df)

    # Print/log the input vector after scaling
    rospy.loginfo(f"Input vector after scaling: {wrench_input_vector_scaled}")

    return wrench_input_vector_scaled
   

def gp_combined_prediction_node():
    """
    ROS node that combines effort prediction and wrench prediction models.
    """
    rospy.init_node('gp_combined_prediction_node')

    # Load model and scaler paths for wrench GP model
    rosbag_name = rospy.get_param('/rosparam/rosbag_name', 'recorded_data.bag')
    data_type = rospy.get_param('/rosparam/data_type', 'wrench')
    rosbag_base_name = os.path.splitext(rosbag_name)[0]

    # Load the GPLVM parameter as a boolean (default is False)
    use_sparse = rospy.get_param('/rosparam/use_sparse', False)  # Default is False
    # Load the K-Fold parameter as a boolean (default is False)
    use_kfold = rospy.get_param('/rosparam/use_kfold', False)  # Default is False

    # Load model and scaler paths for effort and wrench GP models
    model_path = os.path.join('/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/gp_models', data_type)
    scaler_path = os.path.join('/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/gp_models/scalers', data_type)

    # Define suffix based on use of kfold and sparse
    suffix = ""
    if use_kfold and use_sparse:
        suffix = "_k_s"
    elif use_kfold:
        suffix = "_k"
    elif use_sparse:
        suffix = "_s"

    # Define model and scaler paths based on the data type (effort or wrench)
    effort_model_path = '/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/gp_models/effort'
    wrench_model_path = '/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/gp_models/wrench'
    effort_scaler_path = '/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/gp_models/scalers/effort'
    wrench_scaler_path = '/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/gp_models/scalers/wrench'

    # Construct the model and scaler filenames with the appropriate paths
    effort_model_filename = os.path.join(effort_model_path, f"{rosbag_base_name}_effort{suffix}_model.pkl")
    wrench_model_filename = os.path.join(wrench_model_path, f"{rosbag_base_name}_wrench{suffix}_model.pkl")
    effort_X_scaler_filename = os.path.join(effort_scaler_path, f"{rosbag_base_name}_effort_feature_scaler.pkl")
    wrench_X_scaler_filename = os.path.join(wrench_scaler_path, f"{rosbag_base_name}_wrench_feature_scaler.pkl")
    effort_Y_scaler_filename = os.path.join(effort_scaler_path, f"{rosbag_base_name}_effort_target_scaler.pkl")
    wrench_Y_scaler_filename = os.path.join(wrench_scaler_path, f"{rosbag_base_name}_wrench_target_scaler.pkl")


    global effort_gp_model
    effort_gp_model = load_gp_model(effort_model_filename)

    global wrench_gp_model
    wrench_gp_model = load_gp_model(wrench_model_filename)

    global effort_X_scaler, effort_X_feature_names
    effort_X_scaler, effort_X_feature_names = load_X_scaler(effort_X_scaler_filename)

    global wrench_X_scaler, wrench_X_feature_names
    wrench_X_scaler, wrench_X_feature_names = load_X_scaler(wrench_X_scaler_filename)

    global effort_Y_scaler, effort_Y_feature_names
    effort_Y_scaler, effort_Y_feature_names = load_Y_scaler(effort_Y_scaler_filename)

    global wrench_Y_scaler, wrench_Y_feature_names
    wrench_Y_scaler, wrench_Y_feature_names = load_Y_scaler(wrench_Y_scaler_filename)

    # Subscribe to /joint_states and /predicted_effort topics
    rospy.Subscriber('/joint_states', JointState, joint_state_callback)

    # Publisher for wrench prediction
    global wrench_pub
    wrench_pub = rospy.Publisher('/predicted_wrench', WrenchStamped, queue_size=10)

    # Publisher for predicted effort
    global predicted_effort_pub
    predicted_effort_pub = rospy.Publisher('/predicted_effort', Float64MultiArray, queue_size=10)

    rospy.loginfo("Combined GP prediction node is running...")
    rospy.spin()


if __name__ == '__main__':
    try:
        gp_combined_prediction_node()
    except rospy.ROSInterruptException:
        pass
