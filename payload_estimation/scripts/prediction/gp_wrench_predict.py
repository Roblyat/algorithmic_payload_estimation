#!/usr/bin/env python3

import rospy
import pandas as pd
import numpy as np
import GPy
import pickle
import os
from sensor_msgs.msg import JointState  # For /joint_states topic
from geometry_msgs.msg import WrenchStamped  # Wrench message for the predicted forces/torques
from std_msgs.msg import Float64MultiArray  # For /predicted_effort topic

# UR5 joint names to filter from the joint_states message
UR5_JOINTS = ['ur5_elbow_joint', 'ur5_shoulder_lift_joint', 'ur5_shoulder_pan_joint', 
              'ur5_wrist_1_joint', 'ur5_wrist_2_joint', 'ur5_wrist_3_joint']

def load_gp_model(model_filename):
    rospy.loginfo(f"Loading GP model from {model_filename}")
    gp_model = GPy.core.GP.load_model(model_filename)
    rospy.loginfo("GP model loaded successfully.")
    return gp_model

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

def preprocess_input(joint_state_msg, X_scaler, X_feature_names):
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

    input_vector = []
    for i, joint_name in enumerate(UR5_JOINTS):
        position = positions[i]
        velocity = velocities[i]
        effort = efforts[i]
        input_vector.append(position)
        input_vector.append(velocity)
        input_vector.append(effort)

    input_df = pd.DataFrame([input_vector], columns=X_feature_names)
    input_vector_scaled = X_scaler.transform(input_df)
    return input_vector_scaled

def preprocess_predicted_effort(effort_msg, X_scaler, X_feature_names):
    """
    Process the /predicted_effort message which is a Float64MultiArray and contains joint positions, velocities, and efforts.
    """
    input_vector = np.array(effort_msg.data).reshape(1, -1)  # Convert to a 2D array for prediction
    input_df = pd.DataFrame(input_vector, columns=X_feature_names)
    input_vector_scaled = X_scaler.transform(input_df)
    return input_vector_scaled

def predict_with_gp(gp_model, input_vector):
    Y_pred, Y_var = gp_model.predict(input_vector)
    return Y_pred, Y_var

def joint_state_callback(joint_state_msg):
    input_vector = preprocess_input(joint_state_msg, X_scaler, X_feature_names)
    Y_pred, Y_var = predict_with_gp(gp_model, input_vector)

    Y_pred_original = Y_scaler.inverse_transform(Y_pred)  # Inverse scale the predicted values

    publish_wrench_prediction(Y_pred_original)

def predicted_effort_callback(effort_msg):
    """
    Callback for the /predicted_effort topic.
    """
    input_vector = preprocess_predicted_effort(effort_msg, X_scaler, X_feature_names)
    Y_pred, Y_var = predict_with_gp(gp_model, input_vector)

    Y_pred_original = Y_scaler.inverse_transform(Y_pred)  # Inverse scale the predicted values

    publish_wrench_prediction(Y_pred_original)

def publish_wrench_prediction(Y_pred):
    """
    Publish the predicted wrench (force/torque) using a WrenchStamped message.
    """
    output_msg = WrenchStamped()
    output_msg.header.stamp = rospy.Time.now()
    output_msg.wrench.force.x = Y_pred[0][0]  # Predicted Force X
    output_msg.wrench.force.y = Y_pred[0][1]  # Predicted Force Y
    output_msg.wrench.force.z = Y_pred[0][2]  # Predicted Force Z
    output_msg.wrench.torque.x = Y_pred[0][3]  # Predicted Torque X
    output_msg.wrench.torque.y = Y_pred[0][4]  # Predicted Torque Y
    output_msg.wrench.torque.z = Y_pred[0][5]  # Predicted Torque Z

    rospy.loginfo(f"Output Wrench: {output_msg.wrench}")
    prediction_pub.publish(output_msg)

def gp_live_prediction_node():
    rospy.init_node('gp_wrench_prediction_node')

    # Load parameters
    rosbag_name = rospy.get_param('/rosparam/rosbag_name', 'recorded_data.bag')
    data_type = rospy.get_param('/rosparam/data_type', 'wrench')

    if data_type != 'wrench':
        rospy.logerr("This script only supports 'wrench' data. Shutting down.")
        rospy.signal_shutdown("Unsupported data type")
        return

    rosbag_base_name = os.path.splitext(rosbag_name)[0]
    use_kfold = rospy.get_param('/rosparam/use_kfold', False)
    model_path = os.path.join('/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/gp_models', data_type)
    scaler_path = os.path.join('/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/gp_models/scalers', data_type)

    if use_kfold:
        model_filename = os.path.join(model_path, f"{rosbag_base_name}_{data_type}_k_model.pkl.zip")
    else:
        model_filename = os.path.join(model_path, f"{rosbag_base_name}_{data_type}_model.pkl.zip")

    X_scaler_filename = os.path.join(scaler_path, f"{rosbag_base_name}_{data_type}_feature_scaler.pkl")
    Y_scaler_filename = os.path.join(scaler_path, f"{rosbag_base_name}_{data_type}_target_scaler.pkl")

    global gp_model
    gp_model = load_gp_model(model_filename)

    global X_scaler, X_feature_names
    X_scaler, X_feature_names = load_X_scaler(X_scaler_filename)

    global Y_scaler, Y_feature_names
    Y_scaler, Y_feature_names = load_Y_scaler(Y_scaler_filename)

    rospy.Subscriber('/joint_states', JointState, joint_state_callback)
    # rospy.Subscriber('/predicted_effort', Float64MultiArray, predicted_effort_callback)  # New subscriber for predicted effort

    global prediction_pub
    prediction_pub = rospy.Publisher('/predicted_wrench', WrenchStamped, queue_size=10)

    rospy.loginfo("GP live prediction node is running...")
    rospy.spin()

if __name__ == '__main__':
    try:
        gp_live_prediction_node()
    except rospy.ROSInterruptException:
        pass
