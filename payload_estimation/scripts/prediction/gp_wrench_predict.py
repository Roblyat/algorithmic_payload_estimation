#!/usr/bin/env python3

import rospy
import numpy as np
import pickle
import os
from sensor_msgs.msg import JointState  # For /joint_states topic
from geometry_msgs.msg import Wrench  # Optional, for /wrench topic
from std_msgs.msg import Float64MultiArray  # Optional, for publishing predictions

def load_gp_model(model_filename):
    """
    Load the trained GP model from file using pickle.
    """
    rospy.loginfo(f"Loading GP model from {model_filename}")
    with open(model_filename, 'rb') as f:
        gp_model = pickle.load(f)
    rospy.loginfo("GP model loaded successfully.")
    return gp_model

def preprocess_input(joint_state_msg):
    """
    Extract and format the input features from the /joint_states topic message.
    This will create an input vector in the format that matches the GP model's training data.
    """
    positions = joint_state_msg.position
    velocities = joint_state_msg.velocity

    # Build the input vector (modify if needed based on your training set)
    input_vector = []
    for i in range(len(positions)):  # Assuming 6 joints
        input_vector.append(positions[i])
        input_vector.append(velocities[i])
        input_vector.append(velocities[i] / 0.025) #compute acceleration -- delta time 40 hz = 1/40 s

    return np.array([input_vector])  # Return as a 2D array for model input

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
    input_vector = preprocess_input(joint_state_msg)

    # Make a prediction using the GP model
    Y_pred, Y_var = predict_with_gp(gp_model, input_vector)

    # Log or publish the predictions (e.g., efforts or wrench data)
    rospy.loginfo(f"Predicted: {Y_pred}, Variance: {Y_var}")

    #preprocess output method here
    output_vector = []

    for i in range(len(joint_state_msg.position)):
        output_vector.append(joint_state_msg.position[i])
        output_vector.append(joint_state_msg.velocity[i])
        output_vector.append(Y_pred[0][i])

    rospy.loginfo(f"Output: {output_vector}")

    # Publish predictions if needed
    prediction_pub.publish(output_vector)  # Uncomment and adapt for your specific case

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
    model_output_path = os.path.join('/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/gp_models', data_type)
    model_filename = os.path.join(model_output_path, f"{rosbag_base_name}_{data_type}_model.pkl")

    global gp_model
    gp_model = load_gp_model(model_filename)

    # Subscribe to the /joint_states topic to get live data
    rospy.Subscriber('/joint_states', JointState, joint_state_callback)

    # Optional: Publisher for predictions (if you want to publish the predictions)
    global prediction_pub
    prediction_pub = rospy.Publisher('/predicted_effort', Float64MultiArray, queue_size=10)

    rospy.loginfo("GP live prediction node is running...")
    rospy.spin()

if __name__ == '__main__':
    try:
        gp_live_prediction_node()
    except rospy.ROSInterruptException:
        pass