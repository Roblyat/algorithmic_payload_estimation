#!/usr/bin/env python3

import rosbag
import csv
import rospy
import os
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64MultiArray

def rosbag_to_csv(bag_file, joint_states_csv, wrench_csv, predicted_effort_csv, predicted_wrench_csv):
    # Open the ROS bag
    bag = rosbag.Bag(bag_file)

    # Open CSV files
    with open(joint_states_csv, mode='w') as joint_file, \
         open(wrench_csv, mode='w') as wrench_file, \
         open(predicted_effort_csv, mode='w') as effort_file, \
         open(predicted_wrench_csv, mode='w') as pred_wrench_file:
        
        # CSV writers
        joint_writer = csv.writer(joint_file)
        wrench_writer = csv.writer(wrench_file)
        effort_writer = csv.writer(effort_file)
        pred_wrench_writer = csv.writer(pred_wrench_file)

        # Write headers
        joint_writer.writerow(['Time', 'Joint Name', 'Position', 'Velocity', 'Effort'])
        wrench_writer.writerow(['Time', 'Force X', 'Force Y', 'Force Z', 'Torque X', 'Torque Y', 'Torque Z'])
        effort_writer.writerow(['Time', 'Effort Motor 1', 'Effort Motor 2', 'Effort Motor 3', 'Effort Motor 4', 'Effort Motor 5', 'Effort Motor 6'])
        pred_wrench_writer.writerow(['Time', 'Predicted Force X', 'Predicted Force Y', 'Predicted Force Z', 'Predicted Torque X', 'Predicted Torque Y', 'Predicted Torque Z'])

        # Iterate over the messages in the bag
        for topic, msg, t in bag.read_messages(topics=['/joint_states', '/wrench', '/predicted_effort', '/predicted_wrench']):
            time = t.to_sec()  # Get the time in seconds
            
            # Joint states topic
            if topic == '/joint_states':
                for i, name in enumerate(msg.name):
                    joint_writer.writerow([time, name, msg.position[i], msg.velocity[i], msg.effort[i]])

            # Wrench topic
            elif topic == '/wrench':
                force = msg.wrench.force
                torque = msg.wrench.torque
                wrench_writer.writerow([time, force.x, force.y, force.z, torque.x, torque.y, torque.z])

            # Predicted Effort topic (Float64MultiArray)
            elif topic == '/predicted_effort':
                efforts = list(msg.data)  # Convert the tuple to a list
                if len(efforts) >= 6:  # Ensure the array has at least 6 elements
                    effort_writer.writerow([time] + efforts[:6])  # Only take the first 6 elements
            
            # Predicted Wrench topic (WrenchStamped)
            elif topic == '/predicted_wrench':
                pred_force = msg.wrench.force
                pred_torque = msg.wrench.torque
                pred_wrench_writer.writerow([time, pred_force.x, pred_force.y, pred_force.z, pred_torque.x, pred_torque.y, pred_torque.z])

    print(f"Joint states saved to {joint_states_csv}")
    print(f"Wrench data saved to {wrench_csv}")
    print(f"Predicted efforts saved to {predicted_effort_csv}")
    print(f"Predicted wrench data saved to {predicted_wrench_csv}")
    
    bag.close()

if __name__ == "__main__":
    # Rosbag path
    rosbag_path = '/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/raw/rosbag/test'  # Path to the raw training data CSV
    rosbag_name = rospy.get_param('/rosparam/rosbag_name', 'recorded_data.bag')
    bag_file = os.path.join(rosbag_path, rosbag_name)
    
    # Folder path for CSV files
    raw_csv_folder = '/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/raw/csv/test'
    
    # Create filenames for joint states, wrench, predicted effort, and predicted wrench CSV files
    rosbag_base_name = os.path.splitext(rosbag_name)[0]
    joint_states_csv = os.path.join(raw_csv_folder, f"{rosbag_base_name}_jointstates.csv")
    wrench_csv = os.path.join(raw_csv_folder, f"{rosbag_base_name}_wrench.csv")
    predicted_effort_csv = os.path.join(raw_csv_folder, f"{rosbag_base_name}_predicted_effort.csv")
    predicted_wrench_csv = os.path.join(raw_csv_folder, f"{rosbag_base_name}_predicted_wrench.csv")

    # Convert the rosbag data to CSV
    rosbag_to_csv(bag_file, joint_states_csv, wrench_csv, predicted_effort_csv, predicted_wrench_csv)
