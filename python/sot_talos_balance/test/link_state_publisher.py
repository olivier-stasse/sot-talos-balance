import threading
import rospy
from gazebo_msgs.srv import GetLinkState
from dynamic_graph_bridge_msgs.msg import Vector as VectorMsg
from tf.transformations import euler_from_quaternion

def vec2list(v):
    return [v.x,v.y,v.z]

def quat2list(v):
    return [v.x,v.y,v.z,v.w]

class LinkStatePublisher(threading.Thread):
    def __init__(self, link_name, rate, rpy=True, prefix='/sot'):
        super(LinkStatePublisher, self).__init__(name = link_name+"_publisher")
        self.daemon = True
        self.link_name = link_name
        self.rate = rate
        self.prefix = prefix
        self.rpy = rpy
        self._stop = False
        rospy.init_node(self.name, anonymous=True)

    def stop(self):
        self._stop = True

    def stopped(self):
        return self._stop

    def run(self):
        topic_pos = self.prefix + '/' + self.link_name + '/position'
        topic_vel = self.prefix + '/' + self.link_name + '/velocity'

        pub_pos = rospy.Publisher(topic_pos, VectorMsg, queue_size=10)
        pub_vel = rospy.Publisher(topic_vel, VectorMsg, queue_size=10)

        rospy.wait_for_service('/gazebo/get_link_state')
        get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)

        rate = rospy.Rate(self.rate)
        while ( not self.stopped() ) and ( not rospy.is_shutdown() ):
            link_state_msg = get_link_state(link_name = self.link_name)
            link_state = link_state_msg.link_state

            cartesian = vec2list(link_state.pose.position)
            orientation = quat2list(link_state.pose.orientation)
            if self.rpy:
                orientation = list(euler_from_quaternion(orientation,'rzyx'))
            position = cartesian + orientation
            msg_pos = VectorMsg(position)

            velocity = vec2list(link_state.twist.linear) + vec2list(link_state.twist.angular)
            msg_vel = VectorMsg(velocity)

            pub_pos.publish(position)
            pub_vel.publish(velocity)

            rate.sleep()

if __name__=='__main__':
    pub = LinkStatePublisher('base_link',1000)
    pub.start()

