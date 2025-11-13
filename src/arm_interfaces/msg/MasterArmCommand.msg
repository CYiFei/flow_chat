std_msgs/Header header
float32[10] left_command
float32[10] right_command
uint8[10] int_command
bool[32] left_button
bool[32] right_button
bool signal

# 
# left/right_command
# 0-6:1-7轴角度指令
# 7:手柄旋钮行程 
# 
# int_command
# 0:左手柄夹爪行程
# 1:右手柄夹爪行程
# 2:脚踏板指令
# 3:
# 
# signal
# 说明该帧指令是否需要发送