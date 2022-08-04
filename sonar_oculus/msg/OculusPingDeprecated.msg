# oculus simple ping result
# pvt@mit.edu, March 2018 

std_msgs/Header header
sensor_msgs/Image ping                  # the image - contains the header

OculusFire fire_msg

uint32 ping_id
uint32 status
float64    frequency        # acoustic frequency [Hz]
float64    temperature      # external temperature [degC]
float64    pressure         # external pressure [bar]
float64    speed_of_sound   # speed of sound estimate used in range computation [m/s]
uint32     start_time       # 

# datasize
int16[] bearings   # bearings of beams (bearing * PI / 18000)
float64    range_resolution # length of a single range bin
uint32     num_ranges       # number of range lines in the image
uint32     num_beams        # number of bearings in the image     
# uint32     image_offset
# uint32     image_size
# uint32     message_size
