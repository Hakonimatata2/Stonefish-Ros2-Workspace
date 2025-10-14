Header header
uint8 mode                 # default: 0     # 1 for Ping360
uint8 gain_setting         # default: 0     # Analog gain setting (0 = low, 1 = normal, 2 = high)    
int64 transmit_duration    # default: 33    # Units: microsecond; Acoustic transmission duration (1~1000 microseconds)         
int64 sample_period        # default: 200   # Time interval between individual signal intensity samples in 25nsec increments (80 to 40000 == 2 microseconds to 1000 microseconds)\n    
int64 transmit_frequency   # default: 740   # Units: kHz; Acoustic operating frequency. Frequency range is 500kHz to 1000kHz, however it is only practical to use say 650kHz to 850kHz due to the narrow bandwidth of the acoustic receiver.\n        
int64 start_angle          # default: 0     # Units: gradian; Head angle to begin scan sector for autoscan in gradians (0~399 = 0~360 degrees).\n    
int64 stop_angle           # default: 399   # Units: gradian; Head angle to end scan sector for autoscan in gradians (0~399 = 0~360 degrees).\n
int64 angle_step           # default: 1     # Units: gradian; Number of 0.9 degree motor steps between pings for auto scan (1~10 = 0.9~9.0 degrees)\n
int64 delay                # default: 0     # delay: Units: millisecond; An additional delay between successive transmit pulses (0~100 ms). This may be necessary for some programs to avoid collisions on the RS485 USRT.\n
int64 number_of_samples    # default: 1024  # number_of_samples: Number of samples per reflected signal\n
int64 speed_of_sound       # default: 1500  # speed_of_sound: speed of sound in water (1450-1550, 1480 for freshwater, 1520 for salt water at 20 deg)
uint8 continuous           # default: 1     # 0 or 1
int64 intensity_threshold  # default: 150   # Custom field (0-255), used for range estimation 