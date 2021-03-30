# Subsystem Configuration
# 3: 600 kHz 4 beam 20 degree piston
string CEPO 

# CEAUTOSTART On/OFF
# 0	OFF
# 1	DO NOT USE
# 2	RS232
# 3	RS485
# 4	RS422
# 5	EMAC
# 6	UDP
string CEAUTOSTART

# Ensemble output period in HH:MM:SS.hh
string CEI

# Output format
# [1] - RoweTech Binary (RTB)
# [1] - RoweTech DVL (RTD)
# [2] - ASCII Format
# [2] - DVL Format
# [3] - NMEA Status String
# [4] - Ocean Server NMEA DVL Format
# [5] - DVL Format
# [100] - PD0
# [103] - PD3 Binary Format
# [104] - PD4 Binary Format
# [105] - PD5 Binary Format
# [106] - PD6 ASCII Format
# [113] - PD13 ASCII Format
string C485OUT 
string C232OUT 

# 1 - Turn ON Bottom Tracking
# 0 - Turn OFF Bottom Tracking
string CBTON

# Bottom Track Broadband. Sets Bottom Track coded pulse transmission and lag.
# n, m, d, x
# [n] - Transmit Pulse Type and Processing
#     0 = Narrowband.
#     Provides long range profiles at the expense of variance.
#     Not recommended for use with bin size less than the default bin size.
#     1 = Broadband.
#     Typically 15% less range than narrow band but has greatly reduced variance (depending on lag length).
#     Used in conjunction with CWPBP for small bins.
#     2 = Un-coded Broadband (no ambiguity resolver).
#     Non-coded has slightly higher variance than the coded transmit without the annoying autocorrelation side peaks. Better for small bins.
#     4 = Non Coded Broadband pulse-to-pulse (no ambiguity resolver).
#     Provides ultra low variance for small bin sizes.
#     7 = Auto Switch (n=0, n=2, n=4)
# [m] - Lag length in vertical meters (m)
#     The lag length will set the maximum velocity the ADCP can read. A rule of thumb is to set the lag length, where 1.5 is a 'safety factor'.    
# [d] - Long Range Narrowband Auto Switch Depth (m)
#     The depth to switch from Broadband to Narrowband in meters. This is used in the Auto Switch mode.
# [x] - Beam Multiplex
#     1 = ping and process each beam one at a time.
#     2 = ping and process beam pairs.
#     4 = ping and process all four beams together.
string CBTBB

# Set the maximum depth to search for the bottom. This will reduce the amount of time in the search algorithm.
string CBTMX

# Blanking distance for the Bottom Track ping. This is the area that will be ignored just after the ADCP.
# n.nn, m.mm
#     [n.nn] - 0 to 10 meters. Sets the vertical distance from the face of the transducer at which the bottom detection algorithm begins searching for the bottom when range to the bottom is LESS than CBTT parameter b.
#     [m.mm] - 0 to 300 meters. Sets the vertical distance from the face of the transducer at which the bottom detection algorithm begins searching for the bottom when range to the bottom is GREATER than CBTT parameter b.
string CBTBL

# Bottom Track thresholds.
# a,b,c,d
#     [a] - SNR (dB) Shallow Detection Threshold. SNR(dB) shallow detection threshold. Lowering the SNR counts "a" and/or "c" will allow to the DVL to detect smaller bottom echo at greater range. The consequence is that DVL may false detect the bottom at the wrong range when the bottom signal is weak.
#     [b] - Depth(m) Shallow Switch. Depth(m) at which the bottom track switches from using the shallow to the deep SNR. Conditions in shallow water (high backscatter) can be different than deep water so "b" allows for two different SNR settings one for shallow ("a") and one for deep ("c").
#     [c] - SNR(dB) Deep Detection Threshold. SNR(dB) deep detection threshold. Lowering the SNR counts "a" and/or "c" will allow to the DVL to detect smaller bottom echo at greater range. The consequence is that DVL may false detect the bottom at the wrong range when the bottom signal is weak.
#     [d] - Depth(m) Deep Switch. Depth(m) at which the bottom track switches from low to high gain receive. The ADCP/DVL has a high power transmitter. In shallow water the bottom echo may saturate the receiver input. While this does not harm the system saturation limits the measurable signal level of the bottom echo which can make it difficult to detect the bottom in a high water backscatter environment. The ADCP/DVL places the receiver in low gain when the depth is below the "d" parameter setting. The change in gain is about 40 dB. If you observe the ADCP/DVL having difficulty detecting the bottom near the "d" setting you may need set "d" to a deeper or more shallow depth depending on the depth where the detection is poor. A good rule to follow is a strong bottom echo requires a larger value in d and a weak bottom echo a smaller value.
string CBTT

# Not used for bottom tracking only
# This is used to create a pause between the Water Profile and Bottom Track ping. This will allow the Bottom Track ping to die down if it is reverberating and causing interference in the Water Profile ping.
string CBTTBP

# Set thresholds to mark the data bad. This will give 3 options of forcing data to be bad. This can also be used to make all the data good by settings all the thresholds to 0.
# c.cc,q.qq,v.vv
#     [c.cc] - Correlation threshold.
#     [q.qq] - Error Velocity threshold. Range: m/s
#     [v.vv] - Velocity threshold. Range: m/s
string CBTST

# Low pass filtering helps reduce velocity measurement noise
# a,b,c,d,e
# We do not recommend that the filter be enabled.
string CBTFILT

# Water Speed of Sound Control.
# 1, 2, 3, 4
#     [1] - Water Temperature Source
#     [2] - Transducer Depth Source
#     [3] - Salinity Source
#     [4] - Speed of Sound Source
# Choices
#     0 = Command
#     1 = Sensor
#     2 = Internal Calculation.
string CWSSC

# Water Salinity (ppt). Used in the water speed of sound calculation
string CWS

# Water Temperature (degrees celsius). Used in the water speed of sound calculation if the temperature sensor is not available.
string CWT

# Transducer Depth (meters). Used in the water speed of sound calculation.
string CTD

# Water Speed of Sound (meters per second).
string CWSS

# This will set the heading offset for the ADCP. The heading offset will be applied to the heading value before it is to be used within the system for calculations. Typically only the first value is given.
string CHO
# Heading source. Used to state whether to use no heading, internal compass heading or GPS heading. This value should be set to GPS HDT if you cannot calibrate the compass in its environment.
string CHS
# Tilt Source. Set whether to use the tilt from internal compass or no tilt.
string CTS

# Velocity Scale Factor
#  n.nn, m.mm
#     [n.nn] - Water Velocity Scale Factor
#     [m.mm] - Bottom Track Scale Factor
string CVSF
