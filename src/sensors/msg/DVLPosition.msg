# Header to keep track of ROS time and when dead reckoning (position_local) was last reset
Header header
float64 t_last_reset

#Time estimated by the DVL
float64 time

# Distance in X, Y and Z (downward) direction (m)
float64 x
float64 y
float64 z

# Standard deviation (figure of merit) for position (m)
float64 std

# Rotation around X, Y and Z axis (degrees)
float64 roll
float64 pitch
float64 yaw

#Status message
int64 status

#Formatting of json
string format
