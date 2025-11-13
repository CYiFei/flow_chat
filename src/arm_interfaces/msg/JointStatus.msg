float32[10] joint_status
uint8[5] motor_status

# joint_status
# 0:position    力控前位置
# 1:speed       速度
# 2:current     电流
# 3:force       外力
# 4:control     控制量
# 5:tempreture  温度
# 6:admitance   力控后位置
#
# motor_status
# 0:status_code  关节编号（10+左臂，20+右臂，30+夹爪）
# 1:status_code  关节状态码
# 2:error_code   关节错误码
# 3:warning_code 关节警告码
# 