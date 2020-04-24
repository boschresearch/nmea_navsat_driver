# Source: https://www.u-blox.com/sites/default/files/u-blox_ZED-F9P_InterfaceDescription_%28UBX-18010854%29.pdf

# Geofence activation status codes
uint8 INACTIVE=0
uint8 ACTIVE=1

# Geofence state codes
int8 DEACTIVATED=-1
int8 UNKNOWN=0
int8 INSIDE=1
int8 OUTSIDE=2

Header header
# GPS time of week
float32 tow
# Geofence activation status, codes see above
uint8 status
# Number of geofences
uint8 num_fences
# Combined state of all geofences, codes see above
uint8 comb_state
# Individual geofence states, codes see above
int8 fence1
int8 fence2
int8 fence3
int8 fence4
