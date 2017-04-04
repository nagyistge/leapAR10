# A unique ID assigned to this Hand object
# value remains the same across consecutive frames while the tracked hand remains visible
uint32             id
bool               is_left
bool               is_right
bool               is_valid
# How well the internal hand model fits the observed data
# value in the range [0..1]
float64            confidence
# The direction from the palm position toward the fingers.
ros_leap/Vector3   direction
# The normal vector to the palm. If your hand is flat, this vector will point downward, or “out” of the front surface of your palm.
ros_leap/Vector3   palm_normal
# The center position of the palm in millimeters from the Leap Motion Controller origin.
ros_leap/Vector3   palm_position
# The rate of change of the palm position in millimeters/second.
ros_leap/Vector3   palm_velocity
# The average outer width of the hand (not including fingers or thumb) in millimeters.
float64            palm_width
# This sphere is placed roughly as if the hand were holding a ball
ros_leap/Vector3   sphere_center
# The radius of a sphere fit to the curvature of this hand, in millimeters
float64            sphere_radius
# The stabilized palm position of this Hand
ros_leap/Vector3   stabilized_palm_position
# The amount of time this hand has been continuously visible to the Leap Motion controller in seconds
float64            time_visible
# The position of the wrist of this hand
ros_leap/Vector3   wrist_position
ros_leap/Finger[5] fingers
ros_leap/Arm       arm