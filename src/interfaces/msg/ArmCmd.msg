uint8 END_EFF_UNKNOWN=0
uint8 END_EFF_OPEN=1
uint8 END_EFF_CLOSE=2

geometry_msgs/Pose pose
geometry_msgs/Pose current_pose
float32 speed
bool estop
bool reset
uint8 end_effector
bool query_goal_state
float64[] goal_angles
