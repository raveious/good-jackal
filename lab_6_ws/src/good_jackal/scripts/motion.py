import rospy
import std_msgs
from geometry_msgs.msg import Twist
#for camera

def callback(data):
    global
    # PI controller defination

def position_sub():
    rospy.Subscriber('tracked_pos',Int31,callback) #for openCv

def jackal_move():
    pub=rospy.Publisher('cmd_vel',Twist,queue_size=1)
    rospy.init_node('jackal_move',anonymous=True)
    rate=rospy.Rate(50) #1hz
    position_sub()
