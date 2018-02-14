#!/usr/bin/env python
# -*- coding: utf-8 -*-

import arc852.cli_args  as cli
import rospy
from arc852.cli_args import setup_cli_args
from arc852.constants import LOG_LEVEL
from arc852.ros_utils import new_twist
from arc852.utils import setup_logging
from geometry_msgs.msg import Twist


def main():
    # Parse CLI args
    args = setup_cli_args(cli.log_level)

    # Setup logging
    setup_logging(level=args[LOG_LEVEL])

    rospy.init_node('stop_node')

    rospy.loginfo("Running")

    vel_topic = "/cmd_vel"
    rospy.loginfo("Publishing Twist vals to topic {}".format(vel_topic))
    vel_pub = rospy.Publisher(vel_topic, Twist, queue_size=5)
    rospy.sleep(1)

    rospy.loginfo("Stopping the robot")
    vel_pub.publish(new_twist(0, 0))
    rospy.sleep(1)

    rospy.loginfo("Exiting")


if __name__ == '__main__':
    main()
