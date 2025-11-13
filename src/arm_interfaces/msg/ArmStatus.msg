std_msgs/Header header
arm_interfaces/JointStatus[7] left_arm
arm_interfaces/JointStatus[7] right_arm
arm_interfaces/JointStatus[7] gripper
geometry_msgs/Pose left_arm_pose
geometry_msgs/Pose right_arm_pose
uint8[20] other_status


# other_status
# 0:traj_follow_primary_mode 状态码
# 1:traj_follow_secondary_mode
# 2:robot_id_1
# 3:robot_id_2
# 4:driver_status                            0x00 无状态   0x01 报错   0x02 未使能   0x03 使能中
# 5:driver_mode                              0x00 柔顺关   0x01 柔顺开  0x02 拖拽开
# 6:teleop_code
# 7:left_arm_compliance_level                0x00 无   0x01 软   0x02 中   0x03 硬
# 8:right_arm_compliance_level               0x00 无   0x01 软   0x02 中   0x03 硬
# 9:left_gripper_compliance_level            0x00 无   0x01 软   0x02 中   0x03 硬
# 10:right_gripper_compliance_level          0x00 无   0x01 软   0x02 中   0x03 硬
