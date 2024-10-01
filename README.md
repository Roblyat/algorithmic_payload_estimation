# algorithmic_payload_estimation
algorithmic payload estimation ur5

# info
calculate in base_link_frame
always use async moveit methods

#remember
manipulated: ../algorithmic_payload_estimation/manipulator_moveit_config/launch/trajectory_execution.launch.xml
                line 17: <param name="trajectory_execution/allowed_start_tolerance" value="0.5"/> <!-- default 0.01 -->

             ../algorithmic_payload_estimation/manipulator_moveit_config/config/kinematics.yaml
                goal_joint_tolerance: 0.1
                goal_position_tolerance: 0.1
                goal_orientation_tolerance: 0.1

moveit method to try: void 	setRandomTarget ()