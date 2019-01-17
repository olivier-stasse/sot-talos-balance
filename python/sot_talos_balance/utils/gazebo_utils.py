'''This module contains utilities for interacting with Gazebo.'''

import rospy

from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Wrench

def apply_force(force,duration, body_name = "talos::torso_2_link"):
    '''Gazebo service call for applying a force on a body.'''
    rospy.wait_for_service('/gazebo/apply_body_wrench')
    apply_body_wrench_proxy = rospy.ServiceProxy('/gazebo/apply_body_wrench',ApplyBodyWrench)
    wrench          = Wrench()
    wrench.force.x  = force[0]
    wrench.force.y  = force[1]
    wrench.force.z  = force[2]
    wrench.torque.x = 0
    wrench.torque.y = 0
    wrench.torque.z = 0
    apply_body_wrench_proxy(body_name = body_name, wrench = wrench, duration = rospy.Duration(duration))
