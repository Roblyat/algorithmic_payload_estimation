#!/usr/bin/env python3

import rosbag
import csv
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped

def rosbag_to_csv(bag_file, output_csv):
    # Open the ROS bag
    bag = rosbag.Bag(bag_file)

    # Open the CSV file to write
    with open(output_csv, mode='w') as csv_file:
        csv_writer = csv.writer(csv_file)

        # Write the header (depends on the topics you're recording)
        csv_writer.writerow(['Time', 'Joint Name', 'Position', 'Velocity', 'Effort', 'Force X', 'Force Y', 'Force Z', 'Torque X', 'Torque Y', 'Torque Z'])

        # Iterate over the messages in the bag
        for topic, msg, t in bag.read_messages(topics=['/joint_states', '/wrench']):
            if topic == '/joint_states':
                # Extract joint state information
                time = t.to_sec()
                for i, name in enumerate(msg.name):
                    csv_writer.writerow([time, name, msg.position[i], msg.velocity[i], msg.effort[i], '', '', '', '', '', ''])
            elif topic == '/wrench':
                # Extract wrench information
                time = t.to_sec()
                force = msg.wrench.force
                torque = msg.wrench.torque
                csv_writer.writerow([time, '', '', '', '', force.x, force.y, force.z, torque.x, torque.y, torque.z])

    print(f"Converted {bag_file} to {output_csv}")
    bag.close()

if __name__ == "__main__":

    #######################################################################
    ## Replace with the path to your ROS bag and desired CSV output file ##
    #######################################################################
    bag_file = '/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/raw/rosbag/recorded_data.bag'
    output_csv = '/home/robat/catkin_ws/src/algorithmic_payload_estimation/payload_estimation/data/raw/csv/output.csv'

    rosbag_to_csv(bag_file, output_csv)
