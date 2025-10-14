Header header
int64 status                                # status message
int64 error                                 # error message
uint8 operation_mode                        # Operation mode
geometry_msgs/Vector3 accelerometer
float64 fom_ahrs                              # FOM for AHRS
float64 fom_fcl                               # FOM for field calibration
geometry_msgs/Quaternion quaternion   # orientation in quaternions
float64 declination                           # magnetic declination, easterly positive
float64 depth                                 # depth from pressure