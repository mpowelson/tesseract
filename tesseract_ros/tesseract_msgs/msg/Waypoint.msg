int32 JOINT_WAYPOINT=0
int32 JOINT_TOLERANCED_WAYPOINT=1
int32 CARTESIAN_WAYPOINT=2


# Base Class Waypoint
int32 waypoint_type
float64[] coeffs
bool is_critical

# JOINT_WAYPOINT
float64[] joint_positions
string[] joint_names

# JOINT_TOLERANCED_WAYPOINT : public JOINT_WAYPOINT
float64[] upper_tolerance
float64[] lower_tolerance

# CARTESIAN_WAYPOINT
geometry_msgs/Pose cartesian_position
string parent_link

