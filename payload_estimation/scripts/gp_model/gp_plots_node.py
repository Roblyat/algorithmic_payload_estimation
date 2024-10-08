#!/usr/bin/env python3

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from sklearn.metrics import mean_squared_error

# Load data from a CSV file
file_path = '/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/result/results.csv'  # Update with the actual file path
data = pd.read_csv(file_path)

# Plot Actual vs Predicted for Force X, Y, Z
def plot_forces(data):
    fig, axs = plt.subplots(3, 1, figsize=(10, 12))

    # Convert necessary columns to NumPy arrays for plotting
    actual_force_x = np.array(data['Actual Force X'])
    predicted_force_x = np.array(data['Predicted Force X'])

    actual_force_y = np.array(data['Actual Force Y'])
    predicted_force_y = np.array(data['Predicted Force Y'])

    actual_force_z = np.array(data['Actual Force Z'])
    predicted_force_z = np.array(data['Predicted Force Z'])

    # Convert index to a numpy array to ensure compatibility with plotting
    index_array = np.arange(len(data))

    # Force X
    axs[0].plot(index_array, actual_force_x, label='Actual Force X', color='blue')
    axs[0].plot(index_array, predicted_force_x, label='Predicted Force X', color='orange', linestyle='dashed')
    axs[0].set_title('Actual vs Predicted Force X')
    axs[0].set_xlabel('Index')
    axs[0].set_ylabel('Force X')
    axs[0].legend()

    # Force Y
    axs[1].plot(index_array, actual_force_y, label='Actual Force Y', color='blue')
    axs[1].plot(index_array, predicted_force_y, label='Predicted Force Y', color='orange', linestyle='dashed')
    axs[1].set_title('Actual vs Predicted Force Y')
    axs[1].set_xlabel('Index')
    axs[1].set_ylabel('Force Y')
    axs[1].legend()

    # Force Z
    axs[2].plot(index_array, actual_force_z, label='Actual Force Z', color='blue')
    axs[2].plot(index_array, predicted_force_z, label='Predicted Force Z', color='orange', linestyle='dashed')
    axs[2].set_title('Actual vs Predicted Force Z')
    axs[2].set_xlabel('Index')
    axs[2].set_ylabel('Force Z')
    axs[2].legend()

    plt.tight_layout()
    plt.show()

# Plot Actual vs Predicted for Torque X, Y, Z
def plot_torques(data):
    fig, axs = plt.subplots(3, 1, figsize=(10, 12))

    # Convert necessary columns to NumPy arrays for plotting
    actual_torque_x = np.array(data['Actual Torque X'])
    predicted_torque_x = np.array(data['Predicted Torque X'])

    actual_torque_y = np.array(data['Actual Torque Y'])
    predicted_torque_y = np.array(data['Predicted Torque Y'])

    actual_torque_z = np.array(data['Actual Torque Z'])
    predicted_torque_z = np.array(data['Predicted Torque Z'])

    # Convert index to a numpy array
    index_array = np.arange(len(data))

    # Torque X
    axs[0].plot(index_array, actual_torque_x, label='Actual Torque X', color='blue')
    axs[0].plot(index_array, predicted_torque_x, label='Predicted Torque X', color='orange', linestyle='dashed')
    axs[0].set_title('Actual vs Predicted Torque X')
    axs[0].set_xlabel('Index')
    axs[0].set_ylabel('Torque X')
    axs[0].legend()

    # Torque Y
    axs[1].plot(index_array, actual_torque_y, label='Actual Torque Y', color='blue')
    axs[1].plot(index_array, predicted_torque_y, label='Predicted Torque Y', color='orange', linestyle='dashed')
    axs[1].set_title('Actual vs Predicted Torque Y')
    axs[1].set_xlabel('Index')
    axs[1].set_ylabel('Torque Y')
    axs[1].legend()

    # Torque Z
    axs[2].plot(index_array, actual_torque_z, label='Actual Torque Z', color='blue')
    axs[2].plot(index_array, predicted_torque_z, label='Predicted Torque Z', color='orange', linestyle='dashed')
    axs[2].set_title('Actual vs Predicted Torque Z')
    axs[2].set_xlabel('Index')
    axs[2].set_ylabel('Torque Z')
    axs[2].legend()

    plt.tight_layout()
    plt.show()

# Plot error distribution
def plot_error_distribution(data):
    # Calculate errors
    data['Error Force X'] = data['Actual Force X'] - data['Predicted Force X']
    data['Error Force Y'] = data['Actual Force Y'] - data['Predicted Force Y']
    data['Error Force Z'] = data['Actual Force Z'] - data['Predicted Force Z']
    data['Error Torque X'] = data['Actual Torque X'] - data['Predicted Torque X']
    data['Error Torque Y'] = data['Actual Torque Y'] - data['Predicted Torque Y']
    data['Error Torque Z'] = data['Actual Torque Z'] - data['Predicted Torque Z']

    # Plot error distribution
    fig, axs = plt.subplots(2, 1, figsize=(10, 10))
    
    # Errors for Force components
    axs[0].hist([data['Error Force X'], data['Error Force Y'], data['Error Force Z']], 
                label=['Force X Error', 'Force Y Error', 'Force Z Error'], bins=20, alpha=0.7)
    axs[0].set_title('Error Distribution for Force Components')
    axs[0].set_xlabel('Error')
    axs[0].set_ylabel('Frequency')
    axs[0].legend()

    # Errors for Torque components
    axs[1].hist([data['Error Torque X'], data['Error Torque Y'], data['Error Torque Z']], 
                label=['Torque X Error', 'Torque Y Error', 'Torque Z Error'], bins=20, alpha=0.7)
    axs[1].set_title('Error Distribution for Torque Components')
    axs[1].set_xlabel('Error')
    axs[1].set_ylabel('Frequency')
    axs[1].legend()

    plt.tight_layout()
    plt.show()

# Plot Mean Squared Error (MSE) for each component
def plot_mse(data):
    # Calculate MSE
    mse_force_x = mean_squared_error(data['Actual Force X'], data['Predicted Force X'])
    mse_force_y = mean_squared_error(data['Actual Force Y'], data['Predicted Force Y'])
    mse_force_z = mean_squared_error(data['Actual Force Z'], data['Predicted Force Z'])

    mse_torque_x = mean_squared_error(data['Actual Torque X'], data['Predicted Torque X'])
    mse_torque_y = mean_squared_error(data['Actual Torque Y'], data['Predicted Torque Y'])
    mse_torque_z = mean_squared_error(data['Actual Torque Z'], data['Predicted Torque Z'])

    # Bar plot of MSE
    components = ['Force X', 'Force Y', 'Force Z', 'Torque X', 'Torque Y', 'Torque Z']
    mse_values = [mse_force_x, mse_force_y, mse_force_z, mse_torque_x, mse_torque_y, mse_torque_z]

    plt.figure(figsize=(10, 6))
    plt.bar(components, mse_values, color='skyblue')
    plt.title('Mean Squared Error (MSE) for Force and Torque Components')
    plt.ylabel('MSE')
    plt.show()

# Run the plotting functions
plot_forces(data)
plot_torques(data)
plot_error_distribution(data)
plot_mse(data)