# 模块名称，标识消息来源模块
string module_name

# 时间戳，记录消息生成的时间
builtin_interfaces/Time time_stamp

# 消息序列号，用于标识消息的顺序
uint64 message_sequence_number

# 消息代码，表示消息的类型或状态
uint8 message_code

# 任务类型
uint8 TASK_TYPE_UNKNOWN = 0
uint8 TASK_TYPE_MOVEMENT = 1 # 控制机器人按指定方向、距离移动
uint8 TASK_TYPE_MOVE_TO_DES = 2 # 控制机器人移动到指定地点
uint8 TASK_TYPE_ARM_CONTROL = 3 # 控制机器人机械臂展开或收回
uint8 TASK_TYPE_BRACKET_MODE = 4 # 支架模式开启或关闭
uint8 TASK_TYPE_BACK_TO_CHARGE = 5 # 回桩充电
# ... 其他任务类型可以继续添加

# 移动方向
uint8 MOVE_DIRECTION_FORWARD = 1 # 向前
uint8 MOVE_DIRECTION_BACKWARD = 2 # 向后
uint8 MOVE_DIRECTION_LEFT = 3 # 向左
uint8 MOVE_DIRECTION_RIGHT = 4 # 向右