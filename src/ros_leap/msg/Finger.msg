uint32           id
bool             is_valid
# Finger Type
# 0 = TYPE_THUMB
# 1 = TYPE_INDEX
# 2 = TYPE_MIDDLE
# 3 = TYPE_RING
# 4 = TYPE_PINKY
uint8            type
# The estimated width and length of the finger in millimeters.
float64          width
float64          length
ros_leap/Vector3 stabilized_tip_position
float64          time_visible
ros_leap/Bone[4] bones
# The tip position in millimeters from the Leap Motion origin.
ros_leap/Vector3 tip_position
# The rate of change of the tip position in millimeters/second.
ros_leap/Vector3 tip_velocity