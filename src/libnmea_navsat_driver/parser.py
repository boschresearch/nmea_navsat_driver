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

"""Parsing functions for NMEA sentence strings."""

import calendar
import datetime
import logging
import math
import re

logger = logging.getLogger('rosout')


def safe_float(field):
    """Convert  field to a float.

    Args:
        field: The field (usually a str) to convert to float.

    Return:
        The float value represented by field or NaN if float conversion throws a ValueError.
    """
    try:
        return float(field)
    except ValueError:
        return float('NaN')


def safe_int(field):
    """Convert  field to an int.

    Args:
        field: The field (usually a str) to convert to int.

    Return:
        The int value represented by field or 0 if int conversion throws a ValueError.
    """
    try:
        return int(field)
    except ValueError:
        return 0


def convert_latitude(field):
    """Convert a latitude string to floating point decimal degrees.

    Args:
        field (str): Latitude string, expected to be formatted as DDMM.MMM, where
            DD is the latitude degrees, and MM.MMM are the minutes latitude.

    Return:
        Floating point latitude in decimal degrees.
    """
    return safe_float(field[0:2]) + safe_float(field[2:]) / 60.0


def convert_longitude(field):
    """Convert a longitude string to floating point decimal degrees.

    Args:
        field (str): Longitude string, expected to be formatted as DDDMM.MMM, where
            DDD is the longitude degrees, and MM.MMM are the minutes longitude.

    Return:
        Floating point latitude in decimal degrees.
    """
    return safe_float(field[0:3]) + safe_float(field[3:]) / 60.0


def convert_time(nmea_utc):
    """Extract time info from a NMEA UTC time string and use it to generate a UNIX epoch time.

    Time information (hours, minutes, seconds) is extracted from the given string and augmented
    with the date, which is taken from the current system time on the host computer (i.e. UTC now).
    The date ambiguity is resolved by adding a day to the current date if the host time is more than
    12 hours behind the NMEA time and subtracting a day from the current date if the host time is
    more than 12 hours ahead of the NMEA time.

    Args:
        nmea_utc (str): NMEA UTC time string to convert. The expected format is HHMMSS[.SS] where
            HH is the number of hours [0,24), MM is the number of minutes [0,60),
            and SS[.SS] is the number of seconds [0,60) of the time in UTC.

    Return:
        tuple(int, int): 2-tuple of (unix seconds, nanoseconds) if the sentence contains valid time.
        tuple(float, float): 2-tuple of (NaN, NaN) if the sentence does not contain valid time.
    """
    # If one of the time fields is empty, return NaN seconds
    if not nmea_utc[0:2] or not nmea_utc[2:4] or not nmea_utc[4:6]:
        return (float('NaN'), float('NaN'))

    # Get current time in UTC for date information
    utc_time = datetime.datetime.utcnow()
    hours = safe_int(nmea_utc[0:2])
    minutes = safe_int(nmea_utc[2:4])
    seconds = safe_int(nmea_utc[4:6])
    nanosecs = 0
    if len(nmea_utc) > 7:
        nanosecs = safe_int(nmea_utc[7:]) * pow(10, 9 - len(nmea_utc[7:]))

    # Resolve the ambiguity of day
    day_offset = safe_int((utc_time.hour - hours)/12.0)
    utc_time += datetime.timedelta(day_offset)
    utc_time.replace(hour=hours, minute=minutes, second=seconds)

    unix_secs = calendar.timegm(utc_time.timetuple())
    return (unix_secs, nanosecs)


def convert_time_rmc(date_str, time_str):
    """Convert a NMEA RMC date string and time string to UNIX epoch time.

    Args:
        date_str (str): NMEA UTC date string to convert, formatted as DDMMYY.
        nmea_utc (str): NMEA UTC time string to convert. The expected format is HHMMSS.SS where
            HH is the number of hours [0,24), MM is the number of minutes [0,60),
            and SS.SS is the number of seconds [0,60) of the time in UTC.

    Return:
        tuple(int, int): 2-tuple of (unix seconds, nanoseconds) if the sentence contains valid time.
        tuple(float, float): 2-tuple of (NaN, NaN) if the sentence does not contain valid time.
    """
    # If one of the time fields is empty, return NaN seconds
    if not time_str[0:2] or not time_str[2:4] or not time_str[4:6]:
        return (float('NaN'), float('NaN'))

    pc_year = datetime.date.today().year

    # Resolve the ambiguity of century
    """
    example 1: utc_year = 99, pc_year = 2100
    years = 2100 + safe_int((2100 % 100 - 99) / 50.0) = 2099
    example 2: utc_year = 00, pc_year = 2099
    years = 2099 + safe_int((2099 % 100 - 00) / 50.0) = 2100
    """
    utc_year = safe_int(date_str[4:6])
    years = pc_year + safe_int((pc_year % 100 - utc_year) / 50.0)

    months = safe_int(date_str[2:4])
    days = safe_int(date_str[0:2])

    hours = safe_int(time_str[0:2])
    minutes = safe_int(time_str[2:4])
    seconds = safe_int(time_str[4:6])
    nanosecs = safe_int(time_str[7:]) * pow(10, 9 - len(time_str[7:]))

    unix_secs = calendar.timegm((years, months, days, hours, minutes, seconds))
    return (unix_secs, nanosecs)


def convert_status_flag(status_flag):
    """Convert a NMEA RMB/RMC status flag to bool.

    Args:
        status_flag (str): NMEA status flag, which should be "A" or "V"

    Return:
        True if the status_flag is "A" for Active.
    """
    if status_flag == "A":
        return True
    elif status_flag == "V":
        return False
    else:
        return False


def convert_knots_to_mps(knots):
    """Convert a speed in knots to meters per second.

    Args:
        knots (float, int, or str): Speed in knots.

    Return:
        The value of safe_float(knots) converted from knots to meters/second.
    """
    return safe_float(knots) * 0.514444444444


def convert_deg_to_rads(degs):
    """Convert an angle in degrees to radians.

    This wrapper is needed because math.radians doesn't accept non-numeric inputs.

    Args:
        degs (float, int, or str): Angle in degrees

    Return:
        The value of safe_float(degs) converted from degrees to radians.
    """
    return math.radians(safe_float(degs))


"""A dictionary that maps from sentence identifier string (e.g. "GGA") to a list of tuples.
Each tuple is a three-tuple of (str: field name, callable: conversion function, int: field index).
The parser splits the sentence into comma-delimited fields. The string value of each field is passed
to the appropriate conversion function based on the field index."""
parse_maps = {
    "GGA": [
        ("fix_type", int, 6),
        ("latitude", convert_latitude, 2),
        ("latitude_direction", str, 3),
        ("longitude", convert_longitude, 4),
        ("longitude_direction", str, 5),
        ("altitude", safe_float, 9),
        ("mean_sea_level", safe_float, 11),
        ("hdop", safe_float, 8),
        ("num_satellites", safe_int, 7),
        ("utc_time", convert_time, 1),
        ("age_dgps", safe_float, 13),
    ],
    "RMC": [
        ("fix_valid", convert_status_flag, 2),
        ("latitude", convert_latitude, 3),
        ("latitude_direction", str, 4),
        ("longitude", convert_longitude, 5),
        ("longitude_direction", str, 6),
        ("speed", convert_knots_to_mps, 7),
        ("true_course", convert_deg_to_rads, 8),
    ],
    "GST": [
        ("utc_time", safe_float, 1),
        ("ranges_std_dev", safe_float, 2),
        ("semi_major_ellipse_std_dev", safe_float, 3),
        ("semi_minor_ellipse_std_dev", safe_float, 4),
        ("semi_major_orientation", safe_float, 5),
        ("lat_std_dev", safe_float, 6),
        ("lon_std_dev", safe_float, 7),
        ("alt_std_dev", safe_float, 8),
    ],
    "GRS": [
        ("utc_time", safe_float, 1),
        ("mode", safe_int, 2),
        ("residual_01", safe_float, 3),
        ("residual_02", safe_float, 4),
        ("residual_03", safe_float, 5),
        ("residual_04", safe_float, 6),
        ("residual_05", safe_float, 7),
        ("residual_06", safe_float, 8),
        ("residual_07", safe_float, 9),
        ("residual_08", safe_float, 10),
        ("residual_09", safe_float, 11),
        ("residual_10", safe_float, 12),
        ("residual_11", safe_float, 13),
        ("residual_12", safe_float, 14),
        ("system_id", safe_float, 15),
        ("signal_id", safe_float, 16),
    ],
    "HDT": [
        ("heading", safe_float, 1),
    ],
    "VTG": [
        ("true_course", safe_float, 1),
        ("speed", convert_knots_to_mps, 5)
    ],
    "PUBX,00": [
        ("utc_time", convert_time, 2),
        ("latitude", convert_latitude, 3),
        ("latitude_direction", str, 4),
        ("longitude", convert_longitude, 5),
        ("longitude_direction", str, 6),
        ("altitude", safe_float, 7),
        ("nav_stat", str, 8),
        ("h_acc", safe_float, 9),
        ("v_acc", safe_float, 10),
        ("sog", safe_float, 11),
        ("cog", safe_float, 12),
        ("v_vel", safe_float, 13),
        ("diff_age", safe_float, 14),
        ("hdop", safe_float, 15),
        ("vdop", safe_float, 16),
        ("tdop", safe_float, 17),
        ("num_svs", int, 18),
        ("dr", int, 20)
    ],
    "PTNL,VHD": [
        ("utc_time", safe_float, 2),
        ("date", int, 3),
        ("azimuth", safe_float, 4),
        ("azimuth_rate", safe_float, 5),
        ("vertical_angle", safe_float, 6),
        ("vertical_angle_rate", safe_float, 7),
        ("range", safe_float, 8),
        ("range_rate", safe_float, 9),
        ("fix_type", int, 10),
        ("num_satellites", safe_int, 11),
        ("pdop", safe_float, 12)
    ],
    "PTNL,AVR": [
        ("utc_time", convert_time, 2),
        ("yaw", convert_deg_to_rads, 3),
        ("tilt", convert_deg_to_rads, 5),
        ("roll", convert_deg_to_rads, 7),
        ("range", safe_float, 9),
        ("fix_type", safe_int, 10),
        ("pdop", safe_float, 11),
        ("num_satellites", safe_int, 12)
    ]
}


def parse_nmea_sentence(nmea_sentence):
    """Parse a NMEA sentence string into a dictionary.

    Args:
        nmea_sentence (str): A single NMEA sentence of one of the types in parse_maps.

    Return:
        A dict mapping string field names to values for each field in the NMEA sentence or
        False if the sentence could not be parsed.
    """
    # Check for a valid nmea sentence
    if not re.match(
            r'^\$(GP|GN|GL|GB|IN|PTNL|PUBX).*\*[0-9A-Fa-f]{2}$', nmea_sentence):
        logger.debug(
            "Regex didn't match, sentence not valid NMEA? Sentence was: %s" %
            repr(nmea_sentence))
        return False
    fields = [field.strip(',') for field in nmea_sentence[:-3].split(',')]

    # Ignore the $ and talker ID portions (e.g. GP)
    # PTNL is used for Trimble proprietary NMEA messages, their description takes two fields - such as $PTNL,VHD
    # The same holds for u-blox with PUBX-Messages
    if fields[0][1:] not in ["PTNL", "PUBX"]:
        sentence_type = fields[0][3:]
    else:
        sentence_type = fields[0][1:] + "," + fields[1]

    if sentence_type not in parse_maps:
        logger.debug("Sentence type %s not in parse map, ignoring."
                     % repr(sentence_type))
        return False

    parse_map = parse_maps[sentence_type]

    parsed_sentence = {}
    for entry in parse_map:
        parsed_sentence[entry[0]] = entry[1](fields[entry[2]])

    if sentence_type == "RMC":
        parsed_sentence["utc_time"] = convert_time_rmc(fields[9], fields[1])

    return {sentence_type: parsed_sentence}
