export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/zme/workspace/chat_task/src/chat_robot_cpp/third_party/x86_64/webp/lib
source install/setup.bash
ros2 run chat_robot_cpp chat_node --ros-args -p use_audio_input:=true