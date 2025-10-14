Header header                               # 
int32 status                                # status message
int32 error                                 # error message
float32 sound_speed                         # acoustic sound speed in water
float64 temperature                         # environment temperature
float64 pressure                            # external pressure
float32[] beam_velocity                     # velocity of each beam
float32[] beam_distance                     # distance of each beam
float32[] beam_fom                          # Figure of merit (FOM) for each beam (invalid beam when fom = 10.0)
float32[] beam_dt                           # time from center to bottom echo
float32[] beam_time_velocity_est            # pulse length for velocity estimate for each beam
geometry_msgs/Vector3 dvl_velocity_xyz      # XYZ velocity of DVL (invalid when value is -32.768)
geometry_msgs/Vector3 dvl_fom_xyz           # XYZ FOM for DVL (invalid estimate when FOM = 10.0)
float64 dvl_velocity_dt_xyz                 # time from trigger to echo of first beam used in estimate of XYZ-velocity
float64 dvl_time_velocity_estimate_xyz      # pulse length for velocity estimate for XYZ
bool data_valid                             # dvl data valid