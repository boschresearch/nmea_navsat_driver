# Software License Agreement (BSD License)
#
# Copyright (c) 2016-2020, Robert Bosch GmbH
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

"""Defines the class for the nmea_tcp_driver executable."""

import socket
import sys
from time import strftime

import rospy
from libnmea_navsat_driver.driver import RosNMEADriver


class NmeaTcpDriver(object):
    """The nmea_tcp_driver ROS node.

    ROS parameters:
        ~host_ip (str): IP of the TCP device to open. Can be overridden by host_name.
        ~host_name (str): Hostname of the TCP device, optional.
        ~host_port (int): Port on the TCP host to read from.
        ~chunk_size (int): Maximum size in Bytes to be read at once, reads that much or until buffer is empty.
    """

    def __init__(self):
        """Get and process parameters."""
        rospy.init_node('nmea_tcp_driver')
        self.host_ip = rospy.get_param('~ip', '')
        self.host_name = rospy.get_param('~hostname', '')
        self.host_port = rospy.get_param('~port', '')
        self.chunk_size = rospy.get_param('~chunk_size', 1024)
        self.frame_id = RosNMEADriver.get_frame_id()
        self.driver = RosNMEADriver()
        self.logging = rospy.get_param('~logging', False)
        self.logging_path = rospy.get_param('~logging_path', '/tmp/gnss_log_' + str(strftime('%Y-%m-%d_%H-%M-%S')) + '.log')

        # try to resolve hostname, otherwise use IP as fallback
        try:
            # in case the hostname is left empty the given IP shall be used. the
            # empty hostname would otherwise be resolved to 0.0.0.0
            if self.host_name != "":
                self.host_ip = socket.gethostbyname(self.host_name)
                rospy.loginfo("GNSS IP address successfully resolved to \"%s\"", self.host_ip)
        except Exception:
            rospy.logwarn("Unable to resolve GNSS hostname. Fallback to alternative IP address \"%s\"", self.host_ip)

        if self.host_ip == "":
            rospy.logerr("Invalid GNSS IP address")
            sys.exit(1)

        if self.logging:
            try:
                self.log = open(self.logging_path, 'ab')
            except IOError:
                rospy.logwarn("Failed to access logging path.")
                self.logging = False

    def __exit__(self):
        """Clean disconnect on exit."""
        self.disconnect()

        if self.logging:
            self.log.close()

    def connect(self):
        """Connect to device."""
        while not rospy.is_shutdown() and not self.connect_tcp():
            rospy.sleep(5.)

    def connect_tcp(self):
        """Open TCP device."""
        try:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.s.connect((self.host_ip, self.host_port))
            self.s.setblocking(0)
            self.s.settimeout(1.0)
            return True
        except socket.error as e:
            rospy.logerr("GNSS socket error: %s, trying to reconnect", e)
            self.disconnect()
            return False

    def disconnect(self):
        """Try to disconnect cleanly."""
        try:
            self.s.close()
        except AttributeError:
            pass

    def receive(self):
        """Receive and process data."""
        # properly handle too small chunk sizes, rest of previous datablock
        rest = ""

        while not rospy.is_shutdown():
            try:
                chunk = self.s.recv(self.chunk_size)

                for data in chunk.replace("\r", "").split("\n"):
                    # if the datablock doesn't end with a checksum it probably
                    # isn't complete, wait for the rest
                    if len(data) < 3 or data[-3] != "*":
                        rest = rest + data
                        continue
                    if data[-3] == "*":
                        try:
                            self.driver.add_sentence(rest + data, self.frame_id)
                            rest = ""
                        except (ValueError, TypeError):
                            rospy.loginfo("Invalid NMEA message: %s", data)
                        if self.logging:
                            self.log.write(data + '\n')

            except socket.timeout as e:
                # try to reconnect on timeout
                rospy.logerr("GNSS socket error: %s, trying to reconnect", e)
                self.disconnect()
                rospy.loginfo("Trying to reconnect")
                self.connect()

            except socket.error as e:
                self.disconnect()


def main():
    """Create a ROS NMEA Driver and feed it NMEA sentence strings from a TCP device."""
    try:
        nmea_driver = NmeaTcpDriver()
        nmea_driver.connect()
        nmea_driver.receive()
    except rospy.ROSInterruptException:
        rospy.loginfo("Closing TCP connection")
