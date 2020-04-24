# Source: https://www.trimble.com/OEM_ReceiverHelp/V5.11/en/NMEA-0183messages_PTNL_AVR.html
# The heading computation in this message is computed from the moving baseline vector, which requires a two-antenna system.

# GNSS fix type codes
uint8 NO_FIX=0
uint8 AUTONOMOUS=1
uint8 RTK_FLOAT=2
uint8 RTK_FIX=3
uint8 DGPS=4

Header header
# Yaw angle in rad
float32 yaw
# Tilt or pitch angle in rad
float32 tilt
# Roll angle in rad
float32 roll
# Range between both antennas in m
float32 range
# GNSS fix type, codes see above
uint8 status
# Number of satellites in use for position solution
uint8 num_satellites
# 3D position dilution of precision
float32 pdop
