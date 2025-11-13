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


#declear motor mode 
uint8 UNKNOW          = 0
uint8 SPEED_MODE      = 1
uint8 POSITION_MODE   = 2

uint8 chassis_mode 
uint8 rotate_mode
uint8 lift_mode

 
# motor speed feedback
float32 left_speed
float32 right_speed
int32 rotate_speed
int32 lift_speed

# motor position  feedback
int32 left_position
int32 right_position
int32 rotate_position
int32 lift_position



