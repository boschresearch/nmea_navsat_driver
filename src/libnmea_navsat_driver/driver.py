# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Eric Perko
# Copyright (c) 2015-2020, Robert Bosch GmbH
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Provides a driver for NMEA GNSS devices."""

import math

import libnmea_navsat_driver.parser
import libnmea_navsat_driver.parser_ubx
import rospy
from geometry_msgs.msg import QuaternionStamped, TwistStamped
from libnmea_navsat_driver.checksum_utils import check_nmea_checksum
from nav_msgs.msg import Odometry
from nmea_navsat_driver.msg import (NavSatInfo, NavSatTrimbleHeading,
                                    NavSatTrimbleMovingBase,
                                    NavSatUbloxGeoFence,
                                    NavSatUbloxPositionVelocityTime,
                                    NavSatUbloxRelPos,
                                    NavSatUbloxPubxPosition)
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus, TimeReference
from tf.transformations import quaternion_from_euler
from tf import TransformListener


class RosNMEADriver(object):
    """ROS driver for NMEA GNSS devices."""

    def __init__(self):
        """Initialize the ROS NMEA driver.

        :ROS Publishers:
            - NavSatFix publisher on the 'fix' channel.
            - TwistStamped publisher on the 'vel' channel.
            - QuaternionStamped publisher on the 'heading' channel.
            - TimeReference publisher on the 'time_reference' channel.
            - NavSatUbloxRelPos publisher on the 'ublox_relpos' channel.
            - Odometry publisher on the 'ublox_relpos_odom' channel.
            - Imu publisher on the 'ublox_relpos_imu' channel.
            - NavSatUbloxGeoFence publisher on the 'ublox_geofence' channel.
            - NavSatUbloxPositionVelocityTime publisher on the 'ublox_position_velocity_time' channel.
            - NavSatUbloxPubxPosition publisher on the 'ublox_pubx_position' channel.
            - NavSatTrimbleHeading publisher on the 'trimble_heading' channel.
            - Imu publisher on the 'trimble_moving_base_imu' channel.
            - NavSatTrimbleMovingBase publisher on the 'trimble_moving_base' channel.

        :ROS Parameters:
            - ~time_ref_source (str)
                The name of the source in published TimeReference messages. (default None)
            - ~useRMC (bool)
                If true, use RMC NMEA messages. If false, use GGA and VTG messages. (default False)
            - ~epe_quality0 (float)
                Value to use for default EPE quality for fix type 0. (default 1000000)
            - ~epe_quality1 (float)
                Value to use for default EPE quality for fix type 1. (default 4.0)
            - ~epe_quality2 (float)
                Value to use for default EPE quality for fix type 2. (default (0.1)
            - ~epe_quality4 (float)
                Value to use for default EPE quality for fix type 4. (default 0.02)
            - ~epe_quality5 (float)
                Value to use for default EPE quality for fix type 5. (default 1.0)
            - ~epe_quality9 (float)
                Value to use for default EPE quality for fix type 9. (default 3.0)
        """
        self.fix_pub = rospy.Publisher('fix', NavSatFix, queue_size=10)
        self.vel_pub = rospy.Publisher('vel', TwistStamped, queue_size=10)
        self.info_pub = rospy.Publisher('info', NavSatInfo, queue_size=10)
        self.heading_pub = rospy.Publisher('heading', QuaternionStamped, queue_size=10)
        self.use_GNSS_time = rospy.get_param('~use_GNSS_time', False)
        self.use_ublox_messages = rospy.get_param('~use_ublox_messages', False)
        self.use_trimble_messages = rospy.get_param('~use_trimble_messages', False)
        if not self.use_GNSS_time:
            self.time_ref_pub = rospy.Publisher(
                'time_reference', TimeReference, queue_size=10)

        # heading corrections, applied to the following messages
        # * PNTL,AVR
        # * UBX-NAV-RELPOSNED
        self.heading_offset = rospy.get_param('~heading_offset', 0.0)
        self.ccw_heading = False
        # many receivers reports heading north-based in CW direction by default,
        # however most ground-based robotics applications use ENU coordinate
        # frames, requiring east-based CCW heading
        self.heading_enu_frame = rospy.get_param('~heading_enu_frame', True)
        if self.heading_enu_frame:
            self.heading_offset += math.pi/2
            self.ccw_heading = True

        # apply URDF-based heading offset for moving base applications in rad
        if rospy.get_param('~apply_urdf_heading_offset', False):
            try:
                source_frame = rospy.get_param('~heading_urdf_source_frame')
                target_frame = rospy.get_param('~heading_urdf_target_frame')
            except KeyError:
                rospy.logerr("URDF heading offset shall be applied, but URDF frames are not given. Using default values.")
            try:
                self.tf_listener = TransformListener()
                self.tf_listener.waitForTransform(
                    target_frame,
                    source_frame,
                    rospy.Time(0),
                    rospy.Duration(5.0)
                )
                trans, _ = self.tf_listener.lookupTransform(
                    target_frame,
                    source_frame,
                    rospy.Time(0)
                )
                urdf_heading = math.atan2(trans[1], trans[0])
                if self.ccw_heading:
                    urdf_heading *= -1
                self.heading_offset += urdf_heading
                rospy.loginfo("URDF x/y offset {}/{}, applying angular offset of {} deg.".format(
                    trans[0], trans[1], urdf_heading
                ))
            except Exception:
                rospy.logerr("Unable to retrieve URDF transformation between GNSS antennas for heading correction. Using default values.")

        # u-blox messages
        if self.use_ublox_messages:
            self.ublox_relpos_pub = rospy.Publisher('ublox_relpos', NavSatUbloxRelPos, queue_size=10)
            self.ublox_relpos_odom_pub = rospy.Publisher('ublox_relpos_odom', Odometry, queue_size=10)
            self.ublox_relpos_imu_pub = rospy.Publisher('ublox_relpos_imu', Imu, queue_size=10)
            self.ublox_geofence_pub = rospy.Publisher('ublox_geofence', NavSatUbloxGeoFence, queue_size=10)
            self.ublox_position_velocity_time_pub = rospy.Publisher('ublox_position_velocity_time', NavSatUbloxPositionVelocityTime, queue_size=10)
            self.ublox_pubx_position_pub = rospy.Publisher('ublox_pubx_position', NavSatUbloxPubxPosition, queue_size=10)

        # Trimble messages
        if self.use_trimble_messages:
            self.trimble_heading_pub = rospy.Publisher('trimble_heading', NavSatTrimbleHeading, queue_size=10)
            self.trimble_moving_base_pub = rospy.Publisher('trimble_moving_base', NavSatTrimbleMovingBase, queue_size=10)
            self.trimble_moving_base_imu_pub = rospy.Publisher('trimble_moving_base_imu', Imu, queue_size=10)

        self.time_ref_source = rospy.get_param('~time_ref_source', None)
        self.use_RMC = rospy.get_param('~useRMC', False)
        self.valid_fix = False

        # epe = estimated position error
        self.default_epe_quality0 = rospy.get_param('~epe_quality0', 1000000)
        self.default_epe_quality1 = rospy.get_param('~epe_quality1', 4.0)
        self.default_epe_quality2 = rospy.get_param('~epe_quality2', 0.1)
        self.default_epe_quality4 = rospy.get_param('~epe_quality4', 0.02)
        self.default_epe_quality5 = rospy.get_param('~epe_quality5', 1.0)
        self.default_epe_quality9 = rospy.get_param('~epe_quality9', 3.0)
        self.using_receiver_epe = False

        self.lon_std_dev = float("nan")
        self.lat_std_dev = float("nan")
        self.alt_std_dev = float("nan")

        """Format for this dictionary is the fix type from a GGA message as the key, with
        each entry containing a tuple consisting of a default estimated
        position error, a NavSatStatus value, and a NavSatFix covariance value."""
        self.gps_qualities = {
            # Unknown
            -1: [
                self.default_epe_quality0,
                NavSatStatus.STATUS_NO_FIX,
                NavSatFix.COVARIANCE_TYPE_UNKNOWN
            ],
            # Invalid
            0: [
                self.default_epe_quality0,
                NavSatStatus.STATUS_NO_FIX,
                NavSatFix.COVARIANCE_TYPE_UNKNOWN
            ],
            # SPS
            1: [
                self.default_epe_quality1,
                NavSatStatus.STATUS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # DGPS
            2: [
                self.default_epe_quality2,
                NavSatStatus.STATUS_SBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # RTK Fix
            4: [
                self.default_epe_quality4,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # RTK Float
            5: [
                self.default_epe_quality5,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # WAAS
            9: [
                self.default_epe_quality9,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ]
        }

    def add_sentence(self, message, frame_id, timestamp=None, sentence_type="NMEA", ubx_message_definitions=None):
        """Public method to provide a new sentence to the driver.

        Args:
            message (str): NMEA or UBX sentence in string form.
            frame_id (str): TF frame ID of the GPS receiver.
            timestamp (rospy.Time, optional): Time the sentence was received.
                If timestamp is not specified, the current time is used.
            sentence_type (str, optional): NMEA or UBX
            ubx_message_definitions (str, optional): Message definitions for UBX sentences

        Return:
            bool: True if the NMEA string is successfully processed, False if there is an error.
        """
        if not "UBX" == sentence_type:
            if not check_nmea_checksum(message):
                rospy.logwarn("Received a sentence with an invalid checksum. " +
                              "Sentence was: %s" % repr(message))
                return False

            parsed_sentence = libnmea_navsat_driver.parser.parse_nmea_sentence(
                message)
            if not parsed_sentence:
                rospy.logdebug(
                    "Failed to parse NMEA sentence. Sentence was: %s" %
                    message)
                return False
        elif "UBX" == sentence_type:
            parsed_sentence = libnmea_navsat_driver.parser_ubx.parse_ubx_sentence(message, ubx_message_definitions)
            if not parsed_sentence:
                rospy.logdebug("Failed to parse UBX sentence. Sentence was: %s" % message)
                return False

        if timestamp:
            current_time = timestamp
        else:
            current_time = rospy.get_rostime()
        current_fix = NavSatFix()
        current_fix.header.stamp = current_time
        current_fix.header.frame_id = frame_id
        if not self.use_GNSS_time:
            current_time_ref = TimeReference()
            current_time_ref.header.stamp = current_time
            current_time_ref.header.frame_id = frame_id
            if self.time_ref_source:
                current_time_ref.source = self.time_ref_source
            else:
                current_time_ref.source = frame_id

        if not self.use_RMC and 'GGA' in parsed_sentence:
            current_fix.position_covariance_type = \
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED

            data = parsed_sentence['GGA']

            if self.use_GNSS_time:
                if math.isnan(data['utc_time'][0]):
                    rospy.logwarn("Time in the NMEA sentence is NOT valid")
                    return False
                current_fix.header.stamp = rospy.Time(data['utc_time'][0], data['utc_time'][1])

            fix_type = data['fix_type']
            if not (fix_type in self.gps_qualities):
                fix_type = -1
            gps_qual = self.gps_qualities[fix_type]
            default_epe = gps_qual[0]
            current_fix.status.status = gps_qual[1]
            current_fix.position_covariance_type = gps_qual[2]

            if gps_qual > 0:
                self.valid_fix = True
            else:
                self.valid_fix = False

            current_fix.status.service = NavSatStatus.SERVICE_GPS

            latitude = data['latitude']
            if data['latitude_direction'] == 'S':
                latitude = -latitude
            current_fix.latitude = latitude

            longitude = data['longitude']
            if data['longitude_direction'] == 'W':
                longitude = -longitude
            current_fix.longitude = longitude

            # Altitude is above ellipsoid, so adjust for mean-sea-level
            altitude = data['altitude'] + data['mean_sea_level']
            current_fix.altitude = altitude

            # use default epe std_dev unless we've received a GST sentence with
            # epes
            if not self.using_receiver_epe or math.isnan(self.lon_std_dev):
                self.lon_std_dev = default_epe
            if not self.using_receiver_epe or math.isnan(self.lat_std_dev):
                self.lat_std_dev = default_epe
            if not self.using_receiver_epe or math.isnan(self.alt_std_dev):
                self.alt_std_dev = default_epe * 2

            hdop = data['hdop']
            current_fix.position_covariance[0] = (hdop * self.lon_std_dev) ** 2
            current_fix.position_covariance[4] = (hdop * self.lat_std_dev) ** 2
            current_fix.position_covariance[8] = (
                2 * hdop * self.alt_std_dev) ** 2  # FIXME

            self.fix_pub.publish(current_fix)

            if not (math.isnan(data['utc_time'][0]) or self.use_GNSS_time):
                current_time_ref.time_ref = rospy.Time(
                    data['utc_time'][0], data['utc_time'][1])
                self.last_valid_fix_time = current_time_ref
                self.time_ref_pub.publish(current_time_ref)

            info = NavSatInfo()
            info.header.stamp = current_time
            info.header.frame_id = frame_id
            info.num_satellites = data['num_satellites']
            info.last_dgps_update = data['age_dgps']
            self.age_dgps = data['age_dgps']
            info.hdop = data['hdop']
            self.info_pub.publish(info)

        elif not self.use_RMC and 'VTG' in parsed_sentence:
            data = parsed_sentence['VTG']

            # Only report VTG data when you've received a valid GGA fix as
            # well.
            if self.valid_fix:
                current_vel = TwistStamped()
                current_vel.header.stamp = current_time
                current_vel.header.frame_id = frame_id
                current_vel.twist.linear.x = data['speed'] * math.sin(data['true_course'])
                current_vel.twist.linear.y = data['speed'] * math.cos(data['true_course'])
                self.vel_pub.publish(current_vel)

        elif 'RMC' in parsed_sentence:
            data = parsed_sentence['RMC']

            if self.use_GNSS_time:
                if math.isnan(data['utc_time'][0]):
                    rospy.logwarn("Time in the NMEA sentence is NOT valid")
                    return False
                current_fix.header.stamp = rospy.Time(data['utc_time'][0], data['utc_time'][1])

            # Only publish a fix from RMC if the use_RMC flag is set.
            if self.use_RMC:
                if data['fix_valid']:
                    current_fix.status.status = NavSatStatus.STATUS_FIX
                else:
                    current_fix.status.status = NavSatStatus.STATUS_NO_FIX

                current_fix.status.service = NavSatStatus.SERVICE_GPS

                latitude = data['latitude']
                if data['latitude_direction'] == 'S':
                    latitude = -latitude
                current_fix.latitude = latitude

                longitude = data['longitude']
                if data['longitude_direction'] == 'W':
                    longitude = -longitude
                current_fix.longitude = longitude

                current_fix.altitude = float('NaN')
                current_fix.position_covariance_type = \
                    NavSatFix.COVARIANCE_TYPE_UNKNOWN

                self.fix_pub.publish(current_fix)

                if not (math.isnan(data['utc_time'][0]) or self.use_GNSS_time):
                    current_time_ref.time_ref = rospy.Time(
                        data['utc_time'][0], data['utc_time'][1])
                    self.time_ref_pub.publish(current_time_ref)

            # Publish velocity from RMC regardless, since GGA doesn't provide
            # it.
            if data['fix_valid']:
                current_vel = TwistStamped()
                current_vel.header.stamp = current_time
                current_vel.header.frame_id = frame_id
                current_vel.twist.linear.x = data['speed'] * \
                    math.sin(data['true_course'])
                current_vel.twist.linear.y = data['speed'] * \
                    math.cos(data['true_course'])
                self.vel_pub.publish(current_vel)

        elif 'GST' in parsed_sentence:
            data = parsed_sentence['GST']

            # Use receiver-provided error estimate if available
            self.using_receiver_epe = True
            self.lon_std_dev = data['lon_std_dev']
            self.lat_std_dev = data['lat_std_dev']
            self.alt_std_dev = data['alt_std_dev']

        elif 'HDT' in parsed_sentence:
            data = parsed_sentence['HDT']
            if data['heading']:
                current_heading = QuaternionStamped()
                current_heading.header.stamp = current_time
                current_heading.header.frame_id = frame_id
                q = quaternion_from_euler(0, 0, math.radians(data['heading']))
                current_heading.quaternion.x = q[0]
                current_heading.quaternion.y = q[1]
                current_heading.quaternion.z = q[2]
                current_heading.quaternion.w = q[3]
                self.heading_pub.publish(current_heading)

        elif self.use_trimble_messages and 'PTNL,VHD' in parsed_sentence:
            data = parsed_sentence['PTNL,VHD']
            msg = NavSatTrimbleHeading()
            msg.header.stamp = current_time
            msg.header.frame_id = frame_id

            msg.azimuth = data['azimuth']
            msg.azimuth_rate = data['azimuth_rate']
            msg.vertical_angle = data['vertical_angle']
            msg.vertical_angle_rate = data['vertical_angle_rate']
            msg.range = data['range']
            msg.range_rate = data['range_rate']
            # explanation of the status codes in message definition
            msg.status = data['fix_type']
            msg.num_satellites = data['num_satellites']
            msg.pdop = data['pdop']

            self.trimble_heading_pub.publish(msg)

        elif self.use_trimble_messages and 'PTNL,AVR' in parsed_sentence:
            data = parsed_sentence['PTNL,AVR']
            msg = NavSatTrimbleMovingBase()
            msg.header.stamp = current_time
            msg.header.frame_id = frame_id

            msg.yaw = data['yaw']
            if self.ccw_heading:
                msg.yaw = -msg.yaw
            msg.yaw += self.heading_offset
            # wrap yaw angle to [-pi, pi)
            msg.yaw = (msg.yaw + math.pi) % (2 * math.pi) - math.pi
            msg.tilt = data['tilt']
            msg.roll = data['roll']
            msg.range = data['range']
            # explanation of the status codes in message definition
            msg.status = data['fix_type']
            msg.pdop = data['pdop']
            msg.num_satellites = data['num_satellites']

            self.trimble_moving_base_pub.publish(msg)

            # An dual antenna system can only measure tow out of the
            # three angles tilt, roll, yaw. Yaw is always measured.
            # Assuming all not measured angles are zero.
            if math.isnan(msg.tilt):
                msg.tilt = 0.
            if math.isnan(msg.roll):
                msg.roll = 0.
            msg2 = Imu()
            msg2.header.stamp = current_time
            msg2.header.frame_id = frame_id
            q = quaternion_from_euler(msg.tilt, msg.roll, msg.yaw)
            msg2.orientation.x = q[0]
            msg2.orientation.y = q[1]
            msg2.orientation.z = q[2]
            msg2.orientation.w = q[3]

            # Calculate the orientation error estimate based on the position
            # error estimates and the baseline. Very conservative by taking the
            # largest possible error at both ends in both directions. Set high
            # error if there is no proper vector fix (msg.status is not 3).
            if msg.status == 3:
                total_position_error = math.sqrt(self.lat_std_dev ** 2
                                                 + self.lon_std_dev ** 2)
                angular_error_estimate = math.atan2(total_position_error * 2, msg.range)
                angular_error_cov = angular_error_estimate ** 2
            else:
                angular_error_cov = 1

            msg2.orientation_covariance = [angular_error_cov, 0.0, 0.0,
                                           0.0, angular_error_cov, 0.0,
                                           0.0, 0.0, angular_error_cov]
            msg2.angular_velocity_covariance = [-1.0, 0.0, 0.0,
                                                0.0, 0.0, 0.0,
                                                0.0, 0.0, 0.0]
            msg2.linear_acceleration_covariance = [-1.0, 0.0, 0.0,
                                                   0.0, 0.0, 0.0,
                                                   0.0, 0.0, 0.0]
            self.trimble_moving_base_imu_pub.publish(msg2)

        # Documentation source: https://www.u-blox.com/sites/default/files/u-blox_ZED-F9P_InterfaceDescription_%28UBX-18010854%29.pdf
        # Some comments have been directly copied from the documentation
        elif self.use_ublox_messages and 'PUBX,00' in parsed_sentence:
            data = parsed_sentence['PUBX,00']
            msg = NavSatUbloxPubxPosition()
            msg.header.stamp = current_time
            msg.header.frame_id = frame_id

            msg.utc_time = rospy.Time(data['utc_time'][0], data['utc_time'][1])
            msg.latitude = data['latitude']
            if data['latitude_direction'] == 'S':
                msg.latitude = -msg.latitude
            msg.longitude = data['longitude']
            if data['longitude_direction'] == 'W':
                msg.longitude = -msg.longitude
            msg.altitude = data['altitude']
            msg.nav_stat = data['nav_stat']
            msg.h_acc = data['h_acc']
            msg.v_acc = data['v_acc']
            msg.sog = data['sog'] / 3.6
            msg.cog = data['cog'] / 180.0 * math.pi
            msg.v_vel = data['v_vel']
            msg.diff_age = data['diff_age']
            msg.hdop = data['hdop']
            msg.vdop = data['vdop']
            msg.tdop = data['tdop']
            msg.num_svs = data['num_svs']
            msg.dr = data['dr']

            self.ublox_pubx_position_pub.publish(msg)

        # Documentation source: https://www.u-blox.com/sites/default/files/u-blox_ZED-F9P_InterfaceDescription_%28UBX-18010854%29.pdf
        # Some comments have been directly copied from the documentation
        elif self.use_ublox_messages and 'UBX-NAV-RELPOSNED' in parsed_sentence:
            data = parsed_sentence['UBX-NAV-RELPOSNED']
            msg = NavSatUbloxRelPos()
            msg.header.stamp = current_time
            msg.header.frame_id = frame_id

            msg.tow = data['iTOW'] * 0.001
            msg.n = data['relPosN'] * 0.01 + data['relPosHPN'] * 0.0001
            msg.e = data['relPosE'] * 0.01 + data['relPosHPE'] * 0.0001
            msg.d = data['relPosD'] * 0.01 + data['relPosHPD'] * 0.0001
            msg.length = data['relPosLength'] * 0.01 + data['relPosHPLength'] * 0.0001
            msg.heading = data['relPosHeading'] * 0.00001 / 180.0 * math.pi
            if self.ccw_heading:
                msg.heading = -msg.heading
            msg.heading += self.heading_offset
            # wrap yaw angle to [-pi, pi)
            msg.heading = (msg.heading + math.pi) % (2 * math.pi) - math.pi
            msg.acc_n = data['accN'] * 0.0001
            msg.acc_e = data['accE'] * 0.0001
            msg.acc_d = data['accD'] * 0.0001
            msg.acc_length = data['accLength'] * 0.0001
            msg.acc_heading = data['accHeading'] * 0.00001 / 180.0 * math.pi

            data['flags'] = format(data['flags'], '032b')
            msg.rel_pos_heading_valid = int(data['flags'][23], 2)
            msg.ref_obs_miss = int(data['flags'][24], 2)
            msg.ref_pos_miss = int(data['flags'][25], 2)
            msg.is_moving = int(data['flags'][26], 2)
            msg.carr_soln = int(data['flags'][27:29], 2)
            msg.rel_pos_valid = int(data['flags'][29], 2)
            msg.diff_soln = int(data['flags'][30], 2)
            msg.gnss_fix_ok = int(data['flags'][31], 2)

            # accuracy estimates in non RTK fixed modes shouldn't be relied on
            if msg.carr_soln == 2 and msg.rel_pos_valid == 1:
                self.lat_std_dev = msg.acc_n
                self.lon_std_dev = msg.acc_e
                self.alt_std_dev = msg.acc_d
            elif msg.carr_soln == 1 and msg.rel_pos_valid == 1:
                self.lat_std_dev = self.default_epe_quality5
                self.lon_std_dev = self.default_epe_quality5
                self.alt_std_dev = self.default_epe_quality5
            else:
                self.lat_std_dev = self.default_epe_quality1
                self.lon_std_dev = self.default_epe_quality1
                self.alt_std_dev = self.default_epe_quality1

            self.ublox_relpos_pub.publish(msg)

            # report erroneous messages
            if msg.carr_soln != 0:
                if msg.ref_obs_miss == 1 \
                  or msg.ref_pos_miss == 1 \
                  or msg.rel_pos_valid == 0 \
                  or msg.gnss_fix_ok == 0:
                    rospy.logerr("GNSS receiver failed to calculate GNSS fix despite available correction data. Consider lowering the frequency, receiver load might be too high.")
                    return False

            # publish odometry and IMU messages for fusion in case message is fine
            if msg.carr_soln == 0:
                if msg.diff_soln == 1:
                    rospy.logwarn_throttle(10, "Unable to calculate RTK solution.")

                return False

            msg2 = Odometry()
            msg2.header.stamp = current_time
            msg2.header.frame_id = frame_id

            # follow enu conventions
            msg2.pose.pose.position.x = msg.e
            msg2.pose.pose.position.y = msg.n
            msg2.pose.pose.position.z = -msg.d
            msg2.pose.covariance[0] = self.lon_std_dev**2
            msg2.pose.covariance[7] = self.lat_std_dev**2
            msg2.pose.covariance[14] = self.alt_std_dev**2

            # output heading once in moving base mode
            if msg.is_moving == 1:
                # take the already inverted heading from above and not the raw
                # value
                q = quaternion_from_euler(0., 0., msg.heading)
                msg2.pose.pose.orientation.x == q[0]
                msg2.pose.pose.orientation.y == q[1]
                msg2.pose.pose.orientation.z == q[2]
                msg2.pose.pose.orientation.w == q[3]

                # degrade estimated when not in RTK fixed mode
                msg2.pose.covariance[21] = msg.acc_heading**2 + (2 - msg.carr_soln)
                msg2.pose.covariance[28] = msg.acc_heading**2 + (2 - msg.carr_soln)
                msg2.pose.covariance[35] = msg.acc_heading**2 + (2 - msg.carr_soln)
            else:
                msg2.pose.covariance[21] = -1.0
                msg2.pose.covariance[28] = -1.0
                msg2.pose.covariance[35] = -1.0

            self.ublox_relpos_odom_pub.publish(msg2)

            # Publish IMU message if in moving base mode
            if msg.is_moving == 1:
                msg3 = Imu()
                msg3.header.stamp = current_time
                msg3.header.frame_id = frame_id
                xyzw = quaternion_from_euler(0.0, 0.0, msg.heading)
                msg3.orientation.x = xyzw[0]
                msg3.orientation.y = xyzw[1]
                msg3.orientation.z = xyzw[2]
                msg3.orientation.w = xyzw[3]

                if msg.carr_soln == 2:
                    angular_error_cov = msg.acc_heading ** 2
                else:
                    angular_error_cov = msg.acc_heading**2 + (2 - msg.carr_soln)

                msg3.orientation_covariance = [angular_error_cov, 0.0, 0.0,
                                               0.0, angular_error_cov, 0.0,
                                               0.0, 0.0, angular_error_cov]
                msg3.angular_velocity_covariance = [-1.0, 0.0, 0.0,
                                                    0.0, 0.0, 0.0,
                                                    0.0, 0.0, 0.0]
                msg3.linear_acceleration_covariance = [-1.0, 0.0, 0.0,
                                                       0.0, 0.0, 0.0,
                                                       0.0, 0.0, 0.0]
                self.ublox_relpos_imu_pub.publish(msg3)

        elif self.use_ublox_messages and 'UBX-NAV-GEOFENCE' in parsed_sentence:
            data = parsed_sentence['UBX-NAV-GEOFENCE']
            msg = NavSatUbloxGeoFence()
            msg.header.stamp = current_time
            msg.header.frame_id = frame_id

            msg.tow = data['iTOW'] * 0.001
            msg.status = data['status']
            msg.num_fences = data['numFences']
            msg.comb_state = data['combState']
            msg.status = data['status']
            # The original message is variable length, so all fences are
            # initialized with "deactivated" state.
            msg.fence1 = NavSatUbloxGeoFence.DEACTIVATED
            msg.fence2 = NavSatUbloxGeoFence.DEACTIVATED
            msg.fence3 = NavSatUbloxGeoFence.DEACTIVATED
            msg.fence4 = NavSatUbloxGeoFence.DEACTIVATED
            try:
                msg.fence1 = data['state'][0]
                msg.fence2 = data['state'][1]
                msg.fence3 = data['state'][2]
                msg.fence4 = data['state'][3]
            except KeyError:
                pass

            self.ublox_geofence_pub.publish(msg)

        elif self.use_ublox_messages and 'UBX-NAV-PVT' in parsed_sentence:
            data = parsed_sentence['UBX-NAV-PVT']
            msg = NavSatUbloxPositionVelocityTime()
            msg.header.stamp = current_time
            msg.header.frame_id = frame_id

            msg.tow = data['iTOW'] * 0.001
            msg.year = data['year']
            msg.month = data['month']
            msg.day = data['day']
            msg.hour = data['hour']
            msg.min = data['min']
            msg.sec = data['sec']

            bitfield_valid = format(data['valid'], '08b')
            msg.valid_date = int(bitfield_valid[7], 2)
            msg.valid_time = int(bitfield_valid[6], 2)
            msg.fully_resolved = int(bitfield_valid[5], 2)
            msg.valid_mag = int(bitfield_valid[4], 2)

            msg.t_acc = data['tAcc'] / 1.e9
            msg.nano = data['nano']
            msg.fix_type = data['fixType']

            bitfield_flags = format(data['flags'], '08b')
            msg.gnss_fix_ok = int(bitfield_flags[7], 2)
            msg.diff_soln = int(bitfield_flags[6], 2)
            msg.psm_state = int(bitfield_flags[3:6], 2)
            msg.head_veh_valid = int(bitfield_flags[2], 2)
            msg.carr_soln = int(bitfield_flags[0:2], 2)

            bitfield_flags2 = format(data['flags2'], '08b')
            msg.confirmed_avai = int(bitfield_flags2[2], 2)
            msg.confirmed_date = int(bitfield_flags2[1], 2)
            msg.confirmed_time = int(bitfield_flags2[0], 2)

            msg.num_sv = data['numSV']
            msg.lon = data['lon'] / 1.e7
            msg.lat = data['lat'] / 1.e7
            msg.height = data['height'] / 1.e3
            msg.h_msl = data['hMSL'] / 1.e3
            msg.h_acc = data['hAcc'] / 1.e3
            msg.v_acc = data['vAcc'] / 1.e3
            msg.vel_n = data['velN'] / 1.e3
            msg.vel_e = data['velE'] / 1.e3
            msg.vel_d = data['velD'] / 1.e3
            msg.g_speed = data['gSpeed'] / 1.e3
            msg.head_mot = data['headMot'] / 1.e5 / 180.0 * math.pi
            msg.s_acc = data['sAcc'] / 1.e3
            msg.head_acc = data['headAcc'] / 1.e5 / 180.0 * math.pi
            msg.p_dop = data['pDOP'] / 1.e2
            msg.head_veh = data['headVeh'] / 1.e5 / 180.0 * math.pi
            msg.mag_dec = data['magDec'] / 1.e2 / 180.0 * math.pi
            msg.mag_acc = data['magAcc'] / 1.e2 / 180.0 * math.pi

            self.ublox_position_velocity_time_pub.publish(msg)

        else:
            return False

    @staticmethod
    def get_frame_id():
        """Get the TF frame_id.

        Queries rosparam for the ~frame_id param. If a tf_prefix param is set,
        the frame_id is prefixed with the prefix.

        Return:
            str: The fully-qualified TF frame ID.
        """
        frame_id = rospy.get_param('~frame_id', 'gps')
        # Add the TF prefix
        prefix = ""
        prefix_param = rospy.search_param('tf_prefix')
        if prefix_param:
            prefix = rospy.get_param(prefix_param)
            return "%s/%s" % (prefix, frame_id)
        else:
            return frame_id
