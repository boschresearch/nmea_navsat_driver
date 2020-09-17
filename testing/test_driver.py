#!/usr/bin/env python
"""Example testing node for nmea_navsat_driver."""
import subprocess

import rospkg
import rospy
import yaml
from nmea_navsat_driver.msg import NavSatTrimbleMovingBase, NavSatUbloxRelPos
from sensor_msgs.msg import NavSatFix

with open(rospkg.RosPack().get_path('nmea_navsat_driver') + '/testing/config.yaml') as f:
    config = yaml.load(f)


def cb(msg, args):
    """Calback for all messages."""
    global msg_counter
    log, topic = args

    if topic not in msg_counter.keys():
        msg_counter[topic] = 0

    msg_counter[topic] += 1
    check_msg(msg, log, topic)


def check_msg(msg, log, topic):
    """Check whether message content complies with target specs."""
    global errors
    topics = config[log]['topics']

    if topic in topics.keys():
        for attribute in topics[topic]:
            val = getattr(msg, attribute)

            if 'min' in topics[topic][attribute].keys():
                if val < topics[topic][attribute]['min']:
                    errors.append('{} {} {} {}: violating minimium value {}: {}'.format(log, msg.header.stamp.to_sec(), topic, attribute, topics[topic][attribute]['min'], val))
            if 'max' in topics[topic][attribute].keys():
                if val > topics[topic][attribute]['max']:
                    errors.append('{} {} {} {}: violating maximum value {}: {}'.format(log, msg.header.stamp.to_sec(), topic, attribute, topics[topic][attribute]['max'], val))
            if 'val' in topics[topic][attribute].keys():
                if val != topics[topic][attribute]['val']:
                    errors.append('{} {} {} {}: violating value {}: {}'.format(log, msg.header.stamp.to_sec(), topic, attribute, topics[topic][attribute]['val'], val))


if __name__ == '__main__':
    errors = []

    # init ros
    p_roscore = subprocess.Popen(["roscore", "--port", '11311'])
    rospy.init_node('state_machine')

    # process all logs
    for log in config:
        rospy.loginfo("Processing logfile {}".format(log))
        msg_counter = {}

        # subscribers
        s_fix = rospy.Subscriber('/fix', NavSatFix, cb, [log, 'fix'])
        s_trimble_moving_base = rospy.Subscriber('/trimble_moving_base', NavSatTrimbleMovingBase, cb, [log, 'trimble_moving_base'])
        s_ublox_relpos = rospy.Subscriber('/ublox_relpos', NavSatUbloxRelPos, cb, [log, 'ublox_relpos'])

        playback_path = rospkg.RosPack().get_path('nmea_navsat_driver') + '/testing/' + log

        # launch driver
        p_driver = subprocess.Popen(["roslaunch", "nmea_navsat_driver", "nmea_driver_testing.launch", "playback_path:=" + playback_path, "--wait"] + config[log]['parameters'])

        p_driver.communicate()
        s_fix.unregister()
        s_trimble_moving_base.unregister()
        s_ublox_relpos.unregister()

        for topic in config[log]['counter']:
            if msg_counter[topic] != config[log]['counter'][topic]:
                errors.append('{} {}: unmet target message count {}: {}'.format(log, topic, config[log]['counter'][topic], msg_counter[topic]))

    if len(errors) == 0:
        rospy.logwarn('No errors detected')
    else:
        for error in errors:
            rospy.logerr(error)

    p_roscore.terminate()
