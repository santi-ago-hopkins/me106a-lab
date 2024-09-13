#!/usr/bin/env python
import numpy as np
import rospy
from turtle_patrol.srv import Patrol  # Import service type
import sys


def patrol_client():
    # Initialize the client node
    turtle_name = sys.argv[1] + '/turtle_patrol_client'
    rospy.init_node('turtle_patrol_client')
    # Wait until patrol service is ready
    rospy.wait_for_service(turtle_name)
    try:
        # Acquire service proxy
        patrol_proxy = rospy.ServiceProxy(
            turtle_name, Patrol)
        vel = 2.0  # Linear velocity
        omega = 1.0  # Angular velocity
        x = 2.0 # x-position
        y = 2.0 # y-position
        theta = 0.0 # orientation
        rospy.loginfo('Command turtle1 to patrol')
        # Call patrol service via the proxy
        patrol_proxy(vel, omega, x, y, theta)
    except rospy.ServiceException as e:
        rospy.loginfo(e)


if __name__ == '__main__':
    patrol_client()

