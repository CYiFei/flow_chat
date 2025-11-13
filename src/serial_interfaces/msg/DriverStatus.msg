# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
#Frame this data is associated with
std_msgs/Header header

 
# motor speed feedback
float32 left_speed
float32 right_speed
float32 rotate_speed
float32 lift_speed

# motor position  feedback
int32 left_position
int32 right_position
float32 rotate_position
float32 lift_position

# motor current  feedback
float32 left_current
float32 right_current
float32 rotate_current
float32 lift_current

# motor temperature feedback
float32 left_temp
float32 right_temp
float32 rotate_temp
float32 lift_temp

# motor emergency stop state feedback
int16 chassis_status
bool stop_button_state
bool enable_state
bool soft_stop_state
bool hard_stop_state
