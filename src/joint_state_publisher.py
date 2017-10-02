import rospy
import intera_interface
from sensor_msgs.msg import JointState

global pub

def cb(data):
    global pub
    pub.publish(data)

if __name__ == '__main__':
    global pub
    rospy.init_node('joint_state_node')
    sub = rospy.Subscriber('/robot/joint_states', JointState, cb)
    pub = rospy.Publisher('/joint_states', JointState)

    rospy.spin()

