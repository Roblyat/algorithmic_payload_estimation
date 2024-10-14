#!/usr/bin/env python3

import rospy
import os
import pandas as pd
import numpy as np  
import matplotlib.pyplot as plt
from sklearn.metrics import mean_squared_error

# Define a shutdown hook to close all open plots
def shutdown_hook():
    rospy.loginfo("Effort plot node is shutting down. Closing all plots.")
    plt.close('all')  # Close all open matplotlib windows immediately

def handle_close(evt):
    global plot_windows_open
    rospy.loginfo("Plot window closed.")
    plot_windows_open = False

def plot_efforts(data):
    fig, axs = plt.subplots(6, 1, figsize=(10, 18))
    fig.canvas.mpl_connect('close_event', handle_close)
    
    efforts = [(f"Joint {i+1}", f"Actual Effort Joint {i+1}", f"Predicted Effort Joint {i+1}") for i in range(6)]
    
    for i, (joint_label, actual_column, predicted_column) in enumerate(efforts):
        axs[i].plot(np.array(data.index), np.array(data[actual_column]), label=f'Actual {joint_label}', color='blue')
        axs[i].plot(np.array(data.index), np.array(data[predicted_column]), label=f'Predicted {joint_label}', color='orange', linestyle='dashed')
        axs[i].set_title(f'Actual vs Predicted {joint_label}')
        axs[i].set_xlabel('Index')
        axs[i].set_ylabel(f'Effort {joint_label}')
        axs[i].legend()

    plt.tight_layout()
    plt.pause(0.001)

def plot_effort_error_distribution(data):
    data['Error Effort Joint 1'] = data['Actual Effort Joint 1'] - data['Predicted Effort Joint 1']
    data['Error Effort Joint 2'] = data['Actual Effort Joint 2'] - data['Predicted Effort Joint 2']
    data['Error Effort Joint 3'] = data['Actual Effort Joint 3'] - data['Predicted Effort Joint 3']
    data['Error Effort Joint 4'] = data['Actual Effort Joint 4'] - data['Predicted Effort Joint 4']
    data['Error Effort Joint 5'] = data['Actual Effort Joint 5'] - data['Predicted Effort Joint 5']
    data['Error Effort Joint 6'] = data['Actual Effort Joint 6'] - data['Predicted Effort Joint 6']

    fig, axs = plt.subplots(1, 1, figsize=(10, 6))
    fig.canvas.mpl_connect('close_event', handle_close)

    axs.hist([data['Error Effort Joint 1'], data['Error Effort Joint 2'], data['Error Effort Joint 3'],
              data['Error Effort Joint 4'], data['Error Effort Joint 5'], data['Error Effort Joint 6']],
             label=[f'Joint {i+1} Error' for i in range(6)],
             bins=20, alpha=0.7)

    axs.set_title('Error Distribution for Joint Efforts')
    axs.set_xlabel('Error')
    axs.set_ylabel('Frequency')
    axs.legend()

    plt.tight_layout()
    plt.pause(0.001)

def plot_effort_mse(data):
    mse_values = [mean_squared_error(data[f'Actual Effort Joint {i+1}'], data[f'Predicted Effort Joint {i+1}']) for i in range(6)]
    joints = [f'Joint {i+1}' for i in range(6)]

    fig = plt.figure(figsize=(10, 6))
    fig.canvas.mpl_connect('close_event', handle_close)

    plt.bar(joints, mse_values, color='skyblue')
    plt.title('Mean Squared Error (MSE) for Joint Efforts')
    plt.ylabel('MSE')

    plt.pause(0.001)

def main():
    # Initialize the ROS node
    rospy.init_node('effort_plots_node')
    rospy.on_shutdown(shutdown_hook)
    
    data_type = rospy.get_param('/rosparam/data_type', 'effort')  # Default to 'effort'
    if data_type != 'effort':
        rospy.logerr(f"Data type is '{data_type}'. This script only supports 'effort'. Shutting down.")
        rospy.signal_shutdown("Unsupported data type")
        exit()

    rosbag_name = rospy.get_param('/rosparam/rosbag_name', 'recorded_data.bag')
    rosbag_base_name = os.path.splitext(rosbag_name)[0]
    use_kfold = rospy.get_param('/rosparam/use_kfold', False)  # Default is False

    if use_kfold:
        file_path = f'/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/result/{data_type}/{rosbag_base_name}_{data_type}_k_results.csv'
    else:
        file_path = f'/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/result/{data_type}/{rosbag_base_name}_{data_type}_results.csv'

    # Load data
    data = pd.read_csv(file_path)
    
    # Plot functions
    plot_efforts(data)
    plot_effort_error_distribution(data)
    plot_effort_mse(data)

    global plot_windows_open
    plot_windows_open = True

    # Keep ROS node alive to allow for graceful shutdown, loop until plot windows are closed
    while not rospy.is_shutdown() and plot_windows_open:
        if not plt.fignum_exists(1):  # Check if the first figure exists
            rospy.loginfo("All plot windows closed. Exiting...")
            break
        plt.pause(0.1)

if __name__ == '__main__':
    main()