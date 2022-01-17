#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %d", data.data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('Air_IOT_PI4', anonymous=True)

    rospy.Subscriber("Air1_cb", Int8, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()