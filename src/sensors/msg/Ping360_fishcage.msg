Header header
uint8 mode                  = 1     # 1 for Ping360
uint8 gain_setting          = 1     # Analog gain setting (0 = low, 1 = normal, 2 = high)    
int64 transmit_duration     = 1000  # Units: microsecond; Acoustic transmission duration (1~1000 microseconds)         
int64 sample_period         = 80    # Time interval between individual signal intensity samples in 25nsec increments (80 to 40000 == 2 microseconds to 1000 microseconds)\n    
int64 transmit_frequency    = 800   # Units: kHz; Acoustic operating frequency. Frequency range is 500kHz to 1000kHz, however it is only practical to use say 650kHz to 850kHz due to the narrow bandwidth of the acoustic receiver.\n        
int64 start_angle           = 0     # Units: gradian; Head angle to begin scan sector for autoscan in gradians (0~399 = 0~360 degrees).\n    
int64 stop_angle            = 399   # Units: gradian; Head angle to end scan sector for autoscan in gradians (0~399 = 0~360 degrees).\n
int64 num_steps             = 1     # Units: gradian; Number of 0.9 degree motor steps between pings for auto scan (1~10 = 0.9~9.0 degrees)\n
int64 delay                 = 0     # delay: Units: millisecond; An additional delay between successive transmit pulses (0~100 ms). This may be necessary for some programs to avoid collisions on the RS485 USRT.\n
int64 number_of_samples     = 200   # number_of_samples: Number of samples per reflected signal\n
