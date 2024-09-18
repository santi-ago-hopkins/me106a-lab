#!/usr/bin/env python

import numpy as np
import scipy as sp
import kin_func_skeleton as kfs
from sensor_msgs.msg import JointState
import rospy

def baxter_forward_kinematics_from_angles(joint_angles):
    """
    Computes the orientation of the Baxter's left end-effector given the joint
    angles in radians.

    Parameters
    ----------
    joint_angles ((7x) np.ndarray): 7 joint angles (s0, s1, e0, e1, w0, w1, w2)

    Returns
    -------
    (4x4) np.ndarray: homogenous transformation matrix
    """

    qs = np.ndarray((3,8)) # points on each joint axis in the zero configuration
    ws = np.ndarray((3,7)) # axis vector of each joint axis
    
    # Assign the q values
    qs[0:3,0] = [0.0635, 0.2598, 0.1188]
    qs[0:3,1] = [0.1106, 0.3116, 0.3885]
    qs[0:3,2] = [0.1827, 0.3838, 0.3881]
    qs[0:3,3] = [0.3682, 0.5684, 0.3181]
    qs[0:3,4] = [0.4417, 0.6420, 0.3177]
    qs[0:3,5] = [0.6332, 0.8337, 0.3067]
    qs[0:3,6] = [0.7152, 0.9158, 0.3063]
    qs[0:3,7] = [0.7957, 0.9965, 0.3058]

    # Assign the w values
    ws[0:3,0] = [-0.0059,  0.0113,  0.9999]
    ws[0:3,1] = [-0.7077,  0.7065, -0.0122]
    ws[0:3,2] = [ 0.7065,  0.7077, -0.0038]
    ws[0:3,3] = [-0.7077,  0.7065, -0.0122]
    ws[0:3,4] = [ 0.7065,  0.7077, -0.0038]
    ws[0:3,5] = [-0.7077,  0.7065, -0.0122]
    ws[0:3,6] = [ 0.7065,  0.7077, -0.0038]

    R = np.array([[0.0076, 0.0001, -1.0000],
                    [-0.7040, 0.7102, -0.0053],
                    [0.7102, 0.7040, 0.0055]]).T # rotation matrix of zero config

    # YOUR CODE HERE (Task 1)


    #make empty array 4x4
    res = np.eye(4)
    xi_matrix = np.zeros([6,7])

    for i in range(7):
        v = np.cross(-ws[0:3, i], qs[0:3,i])
        xi = np.hstack([v, ws[0:3, i]])
        xi_matrix[:, i] = xi
    
    g = kfs.prod_exp(xi_matrix, joint_angles)
        
    g_st_0 = np.array([[0.0076, 0.0001, -1.0000, 0.7957],
                    [-0.7040, 0.7102, -0.0053, .9965],
                    [0.7102, 0.7040, 0.0055, .3058],
                    [0, 0, 0, 1]])
    
    g = g @ g_st_0
    return g

def baxter_forward_kinematics_from_joint_state(joint_state):
    """
    Computes the orientation of the Baxter's left end-effector given the joint
    state.

    Parameters
    ----------
    joint_state (sensor_msgs.JointState): JointState of Baxter robot

    Returns
    -------
    (4x4) np.ndarray: homogenous transformation matrix
    """
    
    # angles = np.zeros(7)

    # YOUR CODE HERE (Task 2)
    joint_state = joint_state.position
    # s0, s1, e0, e1, w0, w1, w2
    s0 = joint_state[4]
    s1 = joint_state[5]
    e0 = joint_state[2]
    e1 = joint_state[3]
    w0 = joint_state[6]
    w1 = joint_state[7]
    w2 = joint_state[8]
    # s0 = joint_state.position['s0']
    # s1 = joint_state.position['s1']
    # e0 = joint_state['e0']
    # e1 = joint_state['e1']
    # w0 = joint_state['w0']
    # w1 = joint_state['w1']
    # w2 = joint_state['w2']
    angles = [s0, s1, e0, e1, w0, w1, w2]

    print(baxter_forward_kinematics_from_angles(angles))


def listener():
    rospy.Subscriber("robot/joint_states",JointState, baxter_forward_kinematics_from_joint_state)
    rospy.spin()


if __name__ == '__main__':

    # Run this program as a new node in the ROS computation graph called
    # /listener_<id>, where <id> is a randomly generated numeric string. This
    # randomly generated name means we can start multiple copies of this node
    # without having multiple nodes with the same name, which ROS doesn't allow.
    rospy.init_node('forward_kinematics', anonymous=True)

    listener()