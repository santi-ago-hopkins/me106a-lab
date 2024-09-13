#!/usr/bin/env python
from geometry_msgs.msg import Twist
import numpy as np
import rospy
from std_srvs.srv import Empty
from turtle_patrol.srv import Patrol  # Service type
from turtlesim.srv import TeleportAbsolute
import sys


def patrol_callback(request):
    rospy.wait_for_service('clear')
    turtle_name = sys.argv[1]
    rospy.wait_for_service(turtle_name)
    clear_proxy = rospy.ServiceProxy('clear', Empty)
    teleport_proxy = rospy.ServiceProxy(
        turtle_name,
        TeleportAbsolute
    )
    # turtle = request.turtle_name # Turtle name

    vel = request.vel  # Linear velocity
    omega = request.omega  # Angular velocity
    x = request.x # x-position
    y = request.y # y-position
    theta = request.theta # orientation
    pub = rospy.Publisher(
        sys.argv[1], Twist, queue_size=50)
    cmd = Twist()
    cmd.linear.x = vel
    cmd.angular.z = omega
    teleport_proxy(x, y, theta)
    # Publish to cmd_vel at 5 Hz
    rate = rospy.Rate(5)
    # Teleport to initial pose
    teleport_proxy(9, 5, np.pi/2)
    # Clear historical path traces
    clear_proxy()
    while not rospy.is_shutdown():
        pub.publish(cmd)  # Publish to cmd_vel
        rate.sleep()  # Sleep until 
    return cmd  # This line will never be reached

def patrol_server():
    # Initialize the server node for turtle1
    turtle_name = sys.argv[1] + 'turtle_patrol_client'
    rospy.init_node('turtle_patrol_client')
    # Register service
    rospy.Service(
        turtle_name,  # Service name
        Patrol,  # Service type
        patrol_callback  # Service callbackcallback
    )
    rospy.loginfo('Running patrol server...')
    rospy.spin() # Spin the node until Ctrl-C


if __name__ == '__main__':
    patrol_server()

