#!/usr/bin/env python

import tf2_ros
import rospy
import sys

def tf_listener(base, target):
    trans = tfBuffer.lookup_transform(base, target, rospy.Time()) # type: ignore
    print(trans.transform)


if __name__ == '__main__':

    # Run this program as a new node in the ROS computation graph called /talker.
    rospy.init_node('l', anonymous=False) # type: ignore
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)

    # Check if the node has received a signal to shut down. If not, run the
    # talker method.
    while not rospy.is_shutdown():
        try:
            tf_listener(sys.argv[1], sys.argv[2])
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as error:
            print(f'there has been an error : {error}') 

#~/ros_workspaces/lab3/src/forward_kinematics/src       