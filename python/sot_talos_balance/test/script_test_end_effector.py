from sot_talos_balance.utils.run_test_utils import run_test, runCommandClient, evalCommandClient
from time import sleep
import rospy
from gazebo_msgs.srv import ApplyBodyWrench, GetLinkState
from geometry_msgs.msg import Wrench
from dynamic_graph_bridge_msgs.msg import Vector as VectorMsg
from tf.transformations import euler_from_quaternion

rospy.wait_for_service('/gazebo/apply_body_wrench')
apply_body_wrench_proxy = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
wrench          = Wrench()
wrench.force.x  = 15.0 
wrench.force.y  = 0.
wrench.force.z  = 0.
wrench.torque.x = 0.
wrench.torque.y = 0.
wrench.torque.z = 0.
duration = -1
apply_body_wrench_proxy(body_name = 'arm_right_7_link', wrench = wrench, duration = rospy.Duration(duration))
