# Source: https://www.u-blox.com/sites/default/files/u-blox_ZED-F9P_InterfaceDescription_%28UBX-18010854%29.pdf

# GNSS fix type codes
uint8 NO_FIX=0
uint8 DEAD_RECKONING=1
uint8 FIX_2D=2
uint8 FIX_3D=3
uint8 GNSS_AND_DEAD_RECKONING=4
uint8 TIME_FIX=5

# Power Safe Mode state codes
uint8 NOT_ACTIVE=0
uint8 ENABLED=1
uint8 ACQUISITION=2
uint8 TRACKING=3
uint8 POWER_OPTIMIZED=4
uint8 INACTIVE=5

# RTK fix type codes
uint8 NO_RTK=0
uint8 RTK_FLOAT=1
uint8 RTK_FIX=2

Header header
# GPS time of week in ms
float32 tow
# Time in UTC
uint16 year
uint8 month
uint8 day
uint8 hour
uint8 min
uint8 sec
# Valid UTC date
uint8 valid_date
# Valid UTC time
uint8 valid_time
# Time of day fully resolved
uint8 fully_resolved
# Valid magnetic declination
uint8 valid_mag
# Time accuracy estimate in ns
float32 t_acc
# Time in UTC, nanoseconds
int32 nano
# GNSS fix type, codes see above
uint8 fix_type
# Valid GNSS fix
uint8 gnss_fix_ok
# Differential solution
uint8 diff_soln
# Power Safe Mode state, codes see above
uint8 psm_state
# Heading of vehicle is valid
uint8 head_veh_valid
# RTK state, codes see above
uint8 carr_soln
# Information about UTC date and time of day validity confirmation is available
uint8 confirmed_avai
# UTC Date validity could be confirmed
uint8 confirmed_date
# UTC Time of Day could be confirmed
uint8 confirmed_time
# Number of satellites used in Nav solution
uint8 num_sv
# Longitude
float32 lon
# Latitude
float32 lat
# Height above ellipsoid in mm
float32 height
# Height above mean sea level in mm
float32 h_msl
# Horizontal accuracy estimate in mm
float32 h_acc
# Vertical accuracy estimate in mm
float32 v_acc
# NED north velocity in mm/s
float32 vel_n
# NED east velocity in mm/s
float32 vel_e
# NED down velocity in mm/s
float32 vel_d
# Ground speed 2D in mm/s
float32 g_speed
# Heading of motion 2D in rad
float32 head_mot
# Speed accuracy estimate in mm/s
float32 s_acc
# Heading accuracy estimate in rad
float32 head_acc
# 3D position dilution of precision
float32 p_dop
# 2D heading of vehicle in rad
float32 head_veh
# Magnetic declination in rad
float32 mag_dec
# Magnetic declination accuracy in rad
float32 mag_acc
