# Source: https://www.u-blox.com/sites/default/files/u-blox_ZED-F9P_InterfaceDescription_%28UBX-18010854%29.pdf

# RTK fix type codes
uint8 NO_RTK=0
uint8 RTK_FLOAT=1
uint8 RTK_FIX=2

Header header
# GPS time of week in ms
float32 tow
# North component of relative position vector in cm
float32 n
# East component of relative position vector in cm
float32 e
# Down component of relative position vector in cm
float32 d
# Length of the relative position vector in cm
float32 length
# Heading of the relative position vector in rad
float32 heading
# Accuracy of relative position north component in mm
float32 acc_n
# Accuracy of relative position east component in mm
float32 acc_e
# Accuracy of relative position down component in mm
float32 acc_d
# Accuracy of length of the relative position vector in mm
float32 acc_length
# Accuracy of heading of the relative position vector in rad
float32 acc_heading
# A valid fix
uint8 gnss_fix_ok
# Differential corrections were applied
uint8 diff_soln
# Relative position components and accuracies are valid
uint8 rel_pos_valid
# RTK state, codes see above
uint8 carr_soln
# Receiver is operating in moving base mode
uint8 is_moving
# Extrapolated reference position was used to compute moving base solution
uint8 ref_pos_miss
# Extrapolated reference observations were used to compute moving base solution
uint8 ref_obs_miss
# Heading is valid
uint8 rel_pos_heading_valid
