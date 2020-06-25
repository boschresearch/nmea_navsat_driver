# Source: https://www.u-blox.com/sites/default/files/u-blox_ZED-F9P_InterfaceDescription_%28UBX-18010854%29.pdf

# Navigation status codes
string NO_FIX=NF
string DEAD_RECKONING_ONLY_SOLUTION=DR
string STAND_ALONE_2D_SOLUTION=G2
string STAND_ALONE_3D_SOLUTION=G3
string DIFFERENTIAL_2D_SOLUTION=D2
string DIFFERENTIAL_3D_SOLUTION=D3
string COMBINED_GPS_AND_DEAD_RECKONING_SOLUTION=RK
string TIME_ONLY_SOLUTION=TT

Header header
# UTC time
time utc_time
# Latitude
float64 latitude
# Longitude
float64 longitude
# Altitude
float64 altitude
# Navigation status
string nav_stat
# Horizontal accuracy
float32 h_acc
# Vertical accuracy
float32 v_acc
# Speed over ground
float32 sog
# Course over ground
float32 cog
# Vertical velocity
float32 v_vel
# Age of differential corrections
float32 diff_age
# HDOP
float32 hdop
# VDOP
float32 vdop
# TDOP
float32 tdop
# Number of satellites used in the navigation solution
uint8 num_svs
# DR used
uint8 dr
