import rosbag
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Wrench

def process_bag(bag_path):
    bag = rosbag.Bag(bag_path)
    for topic, msg, t in bag.read_messages(topics=['/joint_states', '/wrench']):
        if topic == '/joint_states':
            # Do something with JointState message
            process_joint_state(msg)
        elif topic == '/wrench':
            # Do something with Wrench message
            process_wrench(msg)
    bag.close()

def process_joint_state(msg):
    # Process joint state data
    print(msg.position)

def process_wrench(msg):
    # Process wrench data
    print(msg.force)

if __name__ == '__main__':
    process_bag('/home/robat/recorded_data.bag')
