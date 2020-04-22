# Source: https://www.trimble.com/OEM_ReceiverHelp/V5.11/en/NMEA-0183messages_PTNL_VHD.html
# The heading computation in this message is computed from the moving baseline vector, which requires a two-antenna system.

# GNSS fix type codes
uint8 NO_FIX=0
uint8 AUTONOMOUS=1
uint8 RTK_FLOAT=2
uint8 RTK_FIX=3
uint8 DGPS=4
uint8 SBAS=5
uint8 NETWORK_RTK_FLOAT_3D=6
uint8 NETWORK_RTK_FIX_3D=7
uint8 NETWORK_RTK_FLOAT_2D=8
uint8 NETWORK_RTK_FIX_2D=9
uint8 OMNISTAR_HP_XP=10
uint8 OMNISTAR_VBS=11
uint8 LOCATION_RTK=12
uint8 BEACON_DGPS=13
uint8 CENTERPOINT_RTX=14
uint8 XFILL=15

Header header
# Azimuth angle
float32 azimuth
# Azimuth rate
float32 azimuth_rate
# Vertical angle
float32 vertical_angle
# Vertical angle rate
float32 vertical_angle_rate
# Range between both antennas in m
float32 range
# Range rate
float32 range_rate
# GNSS fix type, codes see above
uint8 status
# Number of satellites in use for vector solution
uint8 num_satellites
# 3D position dilution of precision
float32 pdop
