#!/usr/bin/python
import rospy
from lidar_navigation.msg import Contour
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
import numpy as np
from point2d import Point2D
from threading import Lock, Event, Thread
from arc852.pid_controller import PIDControl
import traceback


class WallTracker(object):

    def __init__(self, turn_direction="left", keeping_distance=0.25):
        # Keeping distance is in meters
        self.keeping_distance = keeping_distance
        self.turn_direction = turn_direction

        rospy.init_node("wall_tracker_node")
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=2)
        self.twist_pub_lock = Lock()

        self.contour_sub = rospy.Subscriber("/contour", Contour, self.on_contour_msg)
        self.contour_lock = Lock()
        self._closest_points = None

        self.keeping_pid = PIDControl(2, 0, 0)
        self.angular_pid = PIDControl(4, 0, 0)
        self.linear_pid = PIDControl(0.3, 0, 0)

        # self.max_linear = 0.5
        # self.max_angular = 3
        # self.keeping_distance_adjustment = 0.5

        self._interrupt = Event()

    def on_contour_msg(self, msg):

        with self.contour_lock:
            if self.turn_direction == "left":
                self._closest_points = [Point2D(p.x, p.y) for p in msg.nearest_points if p.x <= 0]
            else:
                self._closest_points = [Point2D(p.x, p.y) for p in msg.nearest_points if p.x >= 0]

    def _get_closest_point(self):
        """Get the closest point along the side of our turning direction"""
        closest_point = None
        with self.contour_lock:
            points = self._closest_points
        if self._closest_points is None:
            return
        for point in points:
            if closest_point is None or \
                    (point.origin_dist < closest_point.origin_dist):

                closest_point = point
        return closest_point

    def move(self):
        while not self._interrupt.is_set():
            try:
                # TODO Currently tends to drift out because there is no actual distance correction

                # Optimizing for following the wall and staying a distance from it seems to require two different
                # forces, almost like two different pressures. They'll probably both get to a point where they
                # hopefully resonate against each other.

                closest_point = self._get_closest_point()
                # Set the desired heading to be perpendicular to the closest point
                # If left, the ideal position is with the closest point to our direct left (at 90)
                angle_to_closest_point = closest_point.heading

                if self.turn_direction == "left":
                    # Scale the angular so we head in the right direction
                    scaling_factor = (-90 - angle_to_closest_point) / 90.0
                    # angular = scaling_factor * self.max_angular
                    # Use PID to bring us close
                    angular = self.angular_pid.get_pid(scaling_factor)
                    linear = self.linear_pid.get_pid(abs(scaling_factor))
                    # linear = scaling_factor * self.max_linear
                else:
                    scaling_factor = -(90.0 - angle_to_closest_point) / 90.0
                    angular = self.angular_pid.get_pid(scaling_factor)
                    linear = self.linear_pid.get_pid(abs(scaling_factor))
                    # linear = 0
                # heading_angle = -1.0 * closest_point.heading#  + (90 if self.turn_direction == "left" else -90)

                # Also calculate the distance from the wall
                wall_dist_adj = self.keeping_pid.get_pid((self.keeping_distance - closest_point.origin_dist) / self.keeping_distance)

                print("pre-angular", angular)
                print("wall adj", wall_dist_adj)
                # No cleaner way besides just adding them together
                angular += -wall_dist_adj

                print("Angle: ", angle_to_closest_point)
                print("Angular: ", angular)
                print("Current dist to wall: ", closest_point.origin_dist)


                twist = Twist()
                # twist.linear.x = linear
                linear = 0.35
                twist.linear.x = min(linear, 5)
                # if self.turn_direction == "left":
                #     twist.linear.x = abs(heading_angle) * self.max_linear
                # else:
                #     twist.linear.x = abs(180 - heading_angle) * self.max_linear

                # -1 * ((centroid.heading / 90.0) * self.__max_angular)
                # twist.angular.z = -1 * heading_angle / 90.0 * self.max_angular
                twist.angular.z = angular
                if abs(twist.angular.z) > 0.6:
                    twist.linear.x = 0
                # elif twist.angular.z < 0.2:
                #     twist.linear.x = 0.1
                print(twist)

                self.twist_pub.publish(twist)
            except Exception as e:
                traceback.print_exc()

    def stop(self):
        self._interrupt.set()

    def run(self):
        try:
            Thread(target=self.move).start()
            rospy.spin()
        except KeyboardInterrupt:
            self.stop()
            exit(0)


if __name__ == '__main__':

    tracker = WallTracker()
    tracker.run()
