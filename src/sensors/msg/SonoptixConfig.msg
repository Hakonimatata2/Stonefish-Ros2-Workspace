Header header
uint8 streaming_mode            # Default: 2        # 1: Raw, 2: RTSP
uint8 operating_mode            # Default: 0        # Frequency. 0: Auto, 1: HF, 2: LF, 3: LFLR (Low freq. long range)
uint8 trigger_mode              # Default: 0        # 0: internal, 1: external, falling edge, 2: external, rising edge
float32 contrast                # Default: 0        # 0-1
int8    gain                    # Default: 0        # positive/negative gain in dB
bool    mirror_image            # Default: True     # true if image shall be mirrored
int8    autodetect_orientation  # Default: 0        # automatically detect orientation
uint8   colormap_index          # Default: 0        # set colormap to the sonoptix ECHO image