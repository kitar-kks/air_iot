#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import paho.mqtt.subscribe as subscribe

hostname = "34.139.76.224"
port = 1883
auth = {
 'username':'admin',
 'password':'p@ssw0rd'
}

def talker():
    pub = rospy.Publisher('set_time', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = subscribe.simple("air_iot/set_time", hostname=hostname,auth=auth)
        rospy.loginfo(msg.payload)
        pub.publish(msg.payload)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass