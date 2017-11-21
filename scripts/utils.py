import logging

import sys
from geometry_msgs.msg import Twist


def setup_logging(filename=None,
                  filemode="a",
                  stream=sys.stderr,
                  level=logging.INFO,
                  format="%(asctime)s %(name)-10s %(funcName)-10s():%(lineno)i: %(levelname)-6s %(message)s"):
    if filename:
        logging.basicConfig(filename=filename, filemode=filemode, level=level, format=format)
    else:
        logging.basicConfig(stream=stream, level=level, format=format)


def new_twist(linear_x, angular_z):
    t = Twist()
    t.linear.x = linear_x
    t.linear.y = 0
    t.linear.z = 0
    t.angular.x = 0
    t.angular.y = 0
    t.angular.z = angular_z
    return t
