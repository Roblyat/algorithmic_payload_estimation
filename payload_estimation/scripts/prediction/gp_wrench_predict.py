#!/usr/bin/env python3

import rospy
import pandas as pd
import numpy as np
import GPy
import pickle
import os
from sensor_msgs.msg import JointState  # For /joint_states topic
from geometry_msgs.msg import WrenchStamped  # Wrench message for the predicted forces/torques
import pandas as pd

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
    efforts = []

    # Extract only the UR5 joints from the joint_state message
    for joint_name in UR5_JOINTS:
        if joint_name in joint_state_msg.name:
            index = joint_state_msg.name.index(joint_name)
            positions.append(joint_state_msg.position[index])
            velocities.append(joint_state_msg.velocity[index])
            efforts.append(joint_state_msg.effort[index])  # Include effort for wrench prediction

    # Build the input vector based on positions, velocities, and efforts
    input_vector = []
    rospy.loginfo("Joint States for UR5 Joints:")
    for i, joint_name in enumerate(UR5_JOINTS):
        position = positions[i]
        velocity = velocities[i]
        effort = efforts[i]
        
        # Print/log the joint states for each joint
        rospy.loginfo(f"Joint {joint_name}: Position={position}, Velocity={velocity}, Effort={effort}")
        
        # Append the values to the input vector
        input_vector.append(position)
        input_vector.append(velocity)
        input_vector.append(effort)

    # Print/log the complete input vector before scaling
    rospy.loginfo(f"Input vector before scaling: {input_vector}")

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

    # Log or publish the predictions (force and torque data)
    rospy.loginfo(f"Predicted Force/Torque: {Y_pred}, Variance: {Y_var}")

    # Prepare the output message (WrenchStamped)
    output_msg = WrenchStamped()
    output_msg.header.stamp = rospy.Time.now()  # Use the current time

    # Fill in the force and torque from the predicted values (Y_pred)
    output_msg.wrench.force.x = Y_pred[0][0]  # Predicted Force X
    output_msg.wrench.force.y = Y_pred[0][1]  # Predicted Force Y
    output_msg.wrench.force.z = Y_pred[0][2]  # Predicted Force Z
    output_msg.wrench.torque.x = Y_pred[0][3]  # Predicted Torque X
    output_msg.wrench.torque.y = Y_pred[0][4]  # Predicted Torque Y
    output_msg.wrench.torque.z = Y_pred[0][5]  # Predicted Torque Z

    # Log the output wrench
    rospy.loginfo(f"Output Wrench: {output_msg.wrench}")

    # Publish the predictions to /predicted_wrench
    prediction_pub.publish(output_msg)


def gp_live_prediction_node():
    """
    ROS node for live prediction of wrench (force and torque) using Gaussian Process models.
    """
    rospy.init_node('gp_live_prediction_node')

    # Load parameters: rosbag name and data type (wrench)
    rosbag_name = rospy.get_param('/rosparam/rosbag_name', 'recorded_data.bag')
    data_type = rospy.get_param('/rosparam/data_type', 'wrench')  # Default to 'wrench'

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

    # Publisher for wrench predictions
    global prediction_pub
    prediction_pub = rospy.Publisher('/predicted_wrench', WrenchStamped, queue_size=10)

    rospy.loginfo("GP live prediction node is running...")
    rospy.spin()


if __name__ == '__main__':
    try:
        gp_live_prediction_node()
    except rospy.ROSInterruptException:
        pass
