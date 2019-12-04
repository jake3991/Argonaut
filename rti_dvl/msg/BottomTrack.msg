Header header
rti_dvl/Command command

# Sample number
uint64 sample
# Start time of the sample in second
float64 time
# Velocity in m/s in body frame
geometry_msgs/Vector3 velocity
# Orientation (roll, pitch, yaw) at the sample
geometry_msgs/Vector3 orientation
# Temperature in Celsius
float64 temperature
# Depth below transducer in m
float64 altitude
# Pressure in BAR
float64 pressure
