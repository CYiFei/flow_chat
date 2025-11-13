builtin_interfaces/Time stamp
serial_interfaces/Joint[7] left_arm_joints
serial_interfaces/Joint[7] right_arm_joints
geometry_msgs/Pose left_arm_pose
geometry_msgs/Pose right_arm_pose
serial_interfaces/TwoFingerGripper left_gripper
serial_interfaces/TwoFingerGripper right_gripper

# currently ,if any joint status is not 4,error_code is 1,and manipulator will not be controlled .
uint32 error_code

# five defferent modes,refer to srv/TrajectoryFollowing.
uint8 trajectory_following_mode

# if it is 1,means an external controller is controlling manipullator,any commands from orin will not be executed .
uint8 external_controller_running
