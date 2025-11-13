builtin_interfaces/Time stamp
dual_arm_interfaces/Joint[8] left_joint_command
dual_arm_interfaces/Joint[8] right_joint_command

#speed level 1-3
uint8 speed 

# 1 for tool-base, 2 for nody-base
uint8 mode 

# true if pressed
bool left_enable
bool left_grasp
bool left_release
bool left_shift

# true if pressed
bool right_enable
bool right_grasp
bool right_release
bool right_shift

