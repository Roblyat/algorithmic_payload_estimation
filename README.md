# algorithmic_payload_estimation
algorithmic payload estimation ur5

# issues
## currently redundant joint_states. solution: 
rosservice call /controller_manager/switch_controller "{start_controllers: [], stop_controllers: ['joint_state_controller'], strictness: 2}"
## goal joint tolerance to high 
resulting in move_group_goal_status::ABORTED ceck:
https://control.ros.org/rolling/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html
