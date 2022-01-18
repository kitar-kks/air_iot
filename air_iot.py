#!/usr/bin/env python
import paho.mqtt.publish as publish
import paho.mqtt.subscribe as subscribe
import rospy
from std_msgs.msg import Int8
from std_msgs.msg import String

hostname = "34.139.76.224"
port = 1883
auth = {
 'username':'admin',
 'password':'p@ssw0rd'
}

def on_message_print(client, userdata, message):
    print("%s %s" % (message.topic, message.payload))

subscribe.callback(on_message_print, "air_iot/set_time", hostname=hostname, port=port, auth=auth)

def callback_air1_cb(data):
    rospy.loginfo(rospy.get_caller_id() + "Air1_cb %d", data.data)
    publish.single("air_iot/Air1_cb",data.data, hostname=hostname, port=port, auth=auth)

def callback_air1_alarm(data):
    rospy.loginfo(rospy.get_caller_id() + "Air1_alarm %d", data.data)
    publish.single("air_iot/Air1_alarm",data.data, hostname=hostname, port=port, auth=auth)

def callback_air1_low_pressure(data):
    rospy.loginfo(rospy.get_caller_id() + "Air1_low_pressure%d", data.data)
    publish.single("air_iot/Air1_low_pressure",data.data, hostname=hostname, port=port, auth=auth)

def callback_air1_high_pressure(data):
    rospy.loginfo(rospy.get_caller_id() + "Air1_high_pressure %d", data.data)
    publish.single("air_iot/Air1_high_pressure",data.data, hostname=hostname, port=port, auth=auth)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('Air_IOT_PI4', anonymous=True)

    rospy.Subscriber("Air1_cb", Int8, callback_air1_cb)
    rospy.Subscriber("Air1_alarm", Int8, callback_air1_alarm)
    rospy.Subscriber("Air1_low_pressure", Int8, callback_air1_low_pressure)
    rospy.Subscriber("Air1_high_pressure", Int8, callback_air1_high_pressure)

    # print("%s %s" % (msg.topic, msg.payload))
    pub = rospy.Publisher('set_time', String, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        msg = subscribe.simple("air_iot/set_time", hostname=hostname)
        rospy.loginfo(msg.payload)
        pub.publish(msg.payload)
        rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    
    rospy.spin()

if __name__ == '__main__':
    listener()
