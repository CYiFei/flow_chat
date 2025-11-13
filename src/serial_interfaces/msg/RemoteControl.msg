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

serial_interfaces/Joint[7] joints


#vibrate level
uint8 vibrate 

# true if pressed
bool button_enable
bool button_grasp
bool button_release
bool button_enter

# TODO more details .
uint8 msg_type 
uint8 MASTER_MSG_TYPE = 2 

