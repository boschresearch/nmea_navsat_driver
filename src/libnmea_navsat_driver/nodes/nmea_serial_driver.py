# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Eric Perko
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

"""Defines the main method for the nmea_serial_driver executable."""

import re
import struct
from time import strftime

import rospkg
import serial
import yaml

import rospy
from libnmea_navsat_driver.driver import RosNMEADriver
from libnmea_navsat_driver.parser_ubx import check_ubx_checksum


def extract_sentences(line, message_definitions, driver, frame_id):
    """Extract NMEA and binary proprietary u-blox sentences out of a mixed mode ASCII-binary stream.

    Loops over the given data until it can't extract a valid sentence anymore, then returns the rest.
    """
    # restricting NMEA mesages to printable asccii characters, so numbers 32 (space) to 126 (tilde)
    nmea_pattern = re.compile(r'\$(GP|GN|GL|GB|IN|PTNL|PUBX)[ -~]+\*[A-F0-9]{2}')
    # identifier for binary u-blox messages
    ub = struct.pack('<BB', 181, 98)
    line_old = ""

    # try again if last run reduced length of sentence and length still larger than zero
    while (len(line) > 0) and (line_old != line):
        line_old = line
        # extract nmea sentences
        nmea_match = nmea_pattern.search(line)
        if nmea_match:
            sentence = nmea_match.group(0)
            driver.add_sentence(sentence, frame_id)
            line = line.replace(sentence, '')
            continue

        # extract ubx sentences
        start = line.find(ub)
        if start != -1:
            # drop binary trash before the first message identifier
            if start != 0:
                line = line[start:]

            # the message header is 6 bytes long - if the message is not at least 6 bytes long the rest might not be transmitted yet. so wait for it.
            if len(line) >= 6:
                message_number = struct.unpack('>H', line[2:4])[0]
                len_sentence = struct.unpack('<H', line[4:6])[0]
                if message_number in message_definitions:
                    # check whether the messages pretended length matches the expected length
                    if message_definitions[message_number]['minlength'] <= len_sentence <= message_definitions[message_number]['maxlength']:
                        sentence = line[:len_sentence + 8]
                        # only publish sentence if the binary data is long enough, otherwise wait for more to come
                        if len(sentence) - 8 == len_sentence:
                            if check_ubx_checksum(sentence):
                                driver.add_sentence(sentence, frame_id, sentence_type="UBX", ubx_message_definitions=message_definitions)
                            else:
                                rospy.logwarn("Discarding corrupted UBX message with class {} and id {} - invalid checksum"
                                              .format(
                                                struct.unpack('<B', line[2:3])[0],
                                                struct.unpack('<B', line[3:4])[0]
                                              ))
                            line = line[len_sentence + 8:]
                    else:
                        rospy.logwarn("Discarding corrupted UBX message with class {} and id {} - incorrect length. Expected {} to {} bytes, given {} bytes."
                                      .format(
                                        struct.unpack('<B', line[2:3])[0],
                                        struct.unpack('<B', line[3:4])[0],
                                        message_definitions[message_number]['minlength'],
                                        message_definitions[message_number]['maxlength'],
                                        len_sentence
                                      ))
                        line = line[2:]
                else:
                    rospy.loginfo("Discarding unsupported UBX message with class {} and id {}"
                                  .format(
                                    struct.unpack('<B', line[2:3])[0],
                                    struct.unpack('<B', line[3:4])[0]
                                  ))
                    line = line[2:]

    # return rest, might be a yet incomplete sentence
    return line


def main():
    """Create and run the nmea_serial_driver ROS node.

    Creates a ROS NMEA Driver and feeds it NMEA sentence strings from a serial device.

    :ROS Parameters:
        - ~port (str)
            Path of the serial device to open.
        - ~baud (int)
            Baud rate to configure the serial device.
        - ~logging (bool)
            Activates logging of raw data.
        - ~logging_path (str)
            Logging path.
        - ~playback (bool)
            Activates playback of raw data instead of reading from device.
        - ~playback_path (str)
            Playback path.
        - ~chunk_size (int)
            Maximum size in bytes to be read at once, reads that much or until buffer is empty.
        - ~driver_rate (int)
            Rate of the driver im Hz. Low rates reduce system load. Too low rates might lead to a growing queue of unprocessed data.
    """
    rospy.init_node('nmea_serial_driver')

    serial_port = rospy.get_param('~port', '/dev/ttyUSB0')
    serial_baud = rospy.get_param('~baud', 4800)
    frame_id = RosNMEADriver.get_frame_id()
    logging = rospy.get_param('~logging', False)
    logging_path = rospy.get_param('~logging_path', '/tmp/gnss_log_' + frame_id + '_' + str(strftime('%Y-%m-%d_%H-%M-%S')) + '.log')
    playback = rospy.get_param('~playback', False)
    playback_path = rospy.get_param('~playback_path', '')
    chunk_size = rospy.get_param('~chunk_size', 1024)
    driver_rate = rospy.get_param('~driver_rate', 100)

    # get binary message definitions
    ubx_message_definition_file = rospkg.RosPack().get_path("nmea_navsat_driver") + "/src/libnmea_navsat_driver/ubx_messages.yaml"
    with open(ubx_message_definition_file, 'r') as stream:
        ubx_message_definitions = yaml.safe_load(stream)

    if logging:
        try:
            log = open(logging_path, 'ab')
        except IOError:
            rospy.logwarn("Failed to access logging path.")
            logging = False

    try:
        if not playback:
            GPS = serial.Serial(port=serial_port, baudrate=serial_baud, timeout=0, rtscts=1)
        else:
            GPS = open(playback_path, "r")

        try:
            driver = RosNMEADriver()
            cache = ""
            startup_delay_expired = False
            while not rospy.is_shutdown():
                data = GPS.read(chunk_size)

                if playback:
                    # Publishing data before subscribers have a chance to connect leads to non-reproducible behavior. Therefore wait for a second after creating the publishers.
                    if startup_delay_expired is False:
                        rospy.sleep(1)
                        startup_delay_expired = True
                    if data == '':
                        rospy.loginfo("End of file reached, shutting down nmea_navsat_driver.")
                        rospy.signal_shutdown("End of file reached")

                try:
                    cache = extract_sentences(cache + data, ubx_message_definitions, driver, frame_id)

                    rospy.sleep(1./driver_rate)
                except ValueError as e:
                    rospy.logwarn(
                        "Value error, likely due to missing fields in the NMEA message. "
                        "Error was: %s. Please report this issue at "
                        "github.com/ros-drivers/nmea_navsat_driver, including a bag file with the NMEA "
                        "sentences that caused it." %
                        e)
                if logging:
                    log.write(data)

        except (rospy.ROSInterruptException, serial.serialutil.SerialException):
            GPS.close()  # Close GPS serial port
    except serial.SerialException as ex:
        rospy.logfatal(
            "Could not open serial port: I/O error({0}): {1}".format(ex.errno, ex.strerror))
