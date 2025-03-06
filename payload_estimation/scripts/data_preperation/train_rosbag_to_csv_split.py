#!/usr/bin/env python3

import rosbag
import csv
import rospy
import os
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped

def rosbag_to_csv(bag_file, joint_states_csv, wrench_csv):
    # Open the ROS bag
    bag = rosbag.Bag(bag_file)

    # Open the CSV file to write joint states
    with open(joint_states_csv, mode='w') as joint_file:
        joint_writer = csv.writer(joint_file)
        # Write header for joint states
        joint_writer.writerow(['Time', 'Joint Name', 'Position', 'Velocity', 'Effort'])

        # Open the CSV file to write wrench data
        with open(wrench_csv, mode='w') as wrench_file:
            wrench_writer = csv.writer(wrench_file)
            # Write header for wrench data
            wrench_writer.writerow(['Time', 'Force X', 'Force Y', 'Force Z', 'Torque X', 'Torque Y', 'Torque Z'])

            # Iterate over the messages in the bag
            for topic, msg, t in bag.read_messages(topics=['/joint_states', '/wrench']):
                time = t.to_sec()  # Get the time in seconds
                if topic == '/joint_states':
                    # Extract joint state information
                    for i, name in enumerate(msg.name):
                        joint_writer.writerow([time, name, msg.position[i], msg.velocity[i], msg.effort[i]])
                elif topic == '/wrench':
                    # Extract wrench information
                    force = msg.wrench.force
                    torque = msg.wrench.torque
                    wrench_writer.writerow([time, force.x, force.y, force.z, torque.x, torque.y, torque.z])

    print(f"Joint states saved to {joint_states_csv}")
    print(f"Wrench data saved to {wrench_csv}")
    bag.close()

if __name__ == "__main__":

    #rosbag path
    rosbag_path = os.getenv("ROS_BAG_TRAIN_PATH", "/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/raw/rosbag/train") # Path to the raw training data CSV
    # rosbag_path = '/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/raw/rosbag/train'  # Path to the raw training data CSV

    rosbag_name = rospy.get_param('/rosparam/rosbag_name', 'recorded_data.bag')
    bag_file = os.path.join(rosbag_path, rosbag_name)
    
    # Folder path for joint states and wrench CSVs
    # raw_csv_folder = '/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/raw/csv/train'
    raw_csv_folder = os.getenv("ROS_CSV_TRAIN_PATH", "/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/raw/csv/train")
    
    # Create filenames for joint states and wrench CSV files
    rosbag_base_name = os.path.splitext(rosbag_name)[0]
    joint_states_csv = os.path.join(raw_csv_folder, f"{rosbag_base_name}_jointstates.csv")
    wrench_csv = os.path.join(raw_csv_folder, f"{rosbag_base_name}_wrench.csv")

    rosbag_to_csv(bag_file, joint_states_csv, wrench_csv)