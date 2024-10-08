#!/usr/bin/env python3

import pandas as pd
import numpy as np  
import matplotlib.pyplot as plt
from sklearn.metrics import mean_squared_error

# Load data from a CSV file
file_path = '/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/result/effort_results.csv'
data = pd.read_csv(file_path)  # Changed from read_excel to read_csv

# Plot Actual vs Predicted for Efforts of All Joints
def plot_efforts(data):
    fig, axs = plt.subplots(6, 1, figsize=(10, 18))
    
    # Convert necessary columns to NumPy arrays for plotting
    actual_effort_1 = np.array(data['Actual Effort Joint 1'])
    predicted_effort_1 = np.array(data['Predicted Effort Joint 1'])
    
    actual_effort_2 = np.array(data['Actual Effort Joint 2'])
    predicted_effort_2 = np.array(data['Predicted Effort Joint 2'])
    
    actual_effort_3 = np.array(data['Actual Effort Joint 3'])
    predicted_effort_3 = np.array(data['Predicted Effort Joint 3'])

    actual_effort_4 = np.array(data['Actual Effort Joint 4'])
    predicted_effort_4 = np.array(data['Predicted Effort Joint 4'])

    actual_effort_5 = np.array(data['Actual Effort Joint 5'])
    predicted_effort_5 = np.array(data['Predicted Effort Joint 5'])

    actual_effort_6 = np.array(data['Actual Effort Joint 6'])
    predicted_effort_6 = np.array(data['Predicted Effort Joint 6'])

    # Joint 1 Effort
    axs[0].plot(np.array(data.index), actual_effort_1, label='Actual Effort Joint 1', color='blue')
    axs[0].plot(np.array(data.index), predicted_effort_1, label='Predicted Effort Joint 1', color='orange', linestyle='dashed')
    axs[0].set_title('Actual vs Predicted Effort Joint 1')
    axs[0].set_xlabel('Index')
    axs[0].set_ylabel('Effort Joint 1')
    axs[0].legend()

    # Joint 2 Effort
    axs[1].plot(np.array(data.index), actual_effort_2, label='Actual Effort Joint 2', color='blue')
    axs[1].plot(np.array(data.index), predicted_effort_2, label='Predicted Effort Joint 2', color='orange', linestyle='dashed')
    axs[1].set_title('Actual vs Predicted Effort Joint 2')
    axs[1].set_xlabel('Index')
    axs[1].set_ylabel('Effort Joint 2')
    axs[1].legend()

    # Joint 3 Effort
    axs[2].plot(np.array(data.index), actual_effort_3, label='Actual Effort Joint 3', color='blue')
    axs[2].plot(np.array(data.index), predicted_effort_3, label='Predicted Effort Joint 3', color='orange', linestyle='dashed')
    axs[2].set_title('Actual vs Predicted Effort Joint 3')
    axs[2].set_xlabel('Index')
    axs[2].set_ylabel('Effort Joint 3')
    axs[2].legend()

    # Joint 4 Effort
    axs[3].plot(np.array(data.index), actual_effort_4, label='Actual Effort Joint 4', color='blue')
    axs[3].plot(np.array(data.index), predicted_effort_4, label='Predicted Effort Joint 4', color='orange', linestyle='dashed')
    axs[3].set_title('Actual vs Predicted Effort Joint 4')
    axs[3].set_xlabel('Index')
    axs[3].set_ylabel('Effort Joint 4')
    axs[3].legend()

    # Joint 5 Effort
    axs[4].plot(np.array(data.index), actual_effort_5, label='Actual Effort Joint 5', color='blue')
    axs[4].plot(np.array(data.index), predicted_effort_5, label='Predicted Effort Joint 5', color='orange', linestyle='dashed')
    axs[4].set_title('Actual vs Predicted Effort Joint 5')
    axs[4].set_xlabel('Index')
    axs[4].set_ylabel('Effort Joint 5')
    axs[4].legend()

    # Joint 6 Effort
    axs[5].plot(np.array(data.index), actual_effort_6, label='Actual Effort Joint 6', color='blue')
    axs[5].plot(np.array(data.index), predicted_effort_6, label='Predicted Effort Joint 6', color='orange', linestyle='dashed')
    axs[5].set_title('Actual vs Predicted Effort Joint 6')
    axs[5].set_xlabel('Index')
    axs[5].set_ylabel('Effort Joint 6')
    axs[5].legend()

    plt.tight_layout()
    plt.show()

# Plot error distribution for Efforts
def plot_effort_error_distribution(data):
    # Calculate errors for each joint
    data['Error Effort Joint 1'] = data['Actual Effort Joint 1'] - data['Predicted Effort Joint 1']
    data['Error Effort Joint 2'] = data['Actual Effort Joint 2'] - data['Predicted Effort Joint 2']
    data['Error Effort Joint 3'] = data['Actual Effort Joint 3'] - data['Predicted Effort Joint 3']
    data['Error Effort Joint 4'] = data['Actual Effort Joint 4'] - data['Predicted Effort Joint 4']
    data['Error Effort Joint 5'] = data['Actual Effort Joint 5'] - data['Predicted Effort Joint 5']
    data['Error Effort Joint 6'] = data['Actual Effort Joint 6'] - data['Predicted Effort Joint 6']

    # Plot error distribution
    fig, axs = plt.subplots(1, 1, figsize=(10, 6))
    
    # Errors for all Joints
    axs.hist([data['Error Effort Joint 1'], data['Error Effort Joint 2'], data['Error Effort Joint 3'],
              data['Error Effort Joint 4'], data['Error Effort Joint 5'], data['Error Effort Joint 6']], 
             label=['Effort Joint 1 Error', 'Effort Joint 2 Error', 'Effort Joint 3 Error', 
                    'Effort Joint 4 Error', 'Effort Joint 5 Error', 'Effort Joint 6 Error'], 
             bins=20, alpha=0.7)
    
    axs.set_title('Error Distribution for Joint Efforts')
    axs.set_xlabel('Error')
    axs.set_ylabel('Frequency')
    axs.legend()

    plt.tight_layout()
    plt.show()

# Plot Mean Squared Error (MSE) for each joint's effort
def plot_effort_mse(data):
    # Calculate MSE for each joint's effort
    mse_joint_1 = mean_squared_error(data['Actual Effort Joint 1'], data['Predicted Effort Joint 1'])
    mse_joint_2 = mean_squared_error(data['Actual Effort Joint 2'], data['Predicted Effort Joint 2'])
    mse_joint_3 = mean_squared_error(data['Actual Effort Joint 3'], data['Predicted Effort Joint 3'])
    mse_joint_4 = mean_squared_error(data['Actual Effort Joint 4'], data['Predicted Effort Joint 4'])
    mse_joint_5 = mean_squared_error(data['Actual Effort Joint 5'], data['Predicted Effort Joint 5'])
    mse_joint_6 = mean_squared_error(data['Actual Effort Joint 6'], data['Predicted Effort Joint 6'])

    # Bar plot of MSE
    joints = ['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6']
    mse_values = [mse_joint_1, mse_joint_2, mse_joint_3, mse_joint_4, mse_joint_5, mse_joint_6]

    plt.figure(figsize=(10, 6))
    plt.bar(joints, mse_values, color='skyblue')
    plt.title('Mean Squared Error (MSE) for Joint Efforts')
    plt.ylabel('MSE')
    plt.show()

# Run the plotting functions
plot_efforts(data)
plot_effort_error_distribution(data)
plot_effort_mse(data)
