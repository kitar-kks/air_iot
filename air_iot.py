#!/usr/bin/env python
import paho.mqtt.publish as publish
import rospy
from std_msgs.msg import Int8
from std_msgs.msg import String
from std_msgs.msg import Float32

hostname = "34.139.76.224"
port = 1883
auth = {
 'username':'admin',
 'password':'p@ssw0rd'
}

# Air1
def callback_air1_cb(data):
    # rospy.loginfo(rospy.get_caller_id() + "Air1_cb %d", data.data)
    publish.single("air_iot/Air1_cb",data.data, hostname=hostname, port=port, auth=auth)

def callback_air1_alarm(data):
    # rospy.loginfo(rospy.get_caller_id() + "Air1_alarm %d", data.data)
    publish.single("air_iot/Air1_alarm",data.data, hostname=hostname, port=port, auth=auth)

def callback_air1_low_pressure(data):
    # rospy.loginfo(rospy.get_caller_id() + "Air1_low_pressure%d", data.data)
    publish.single("air_iot/Air1_low_pressure",data.data, hostname=hostname, port=port, auth=auth)

def callback_air1_high_pressure(data):
    # rospy.loginfo(rospy.get_caller_id() + "Air1_high_pressure %d", data.data)
    publish.single("air_iot/Air1_high_pressure",data.data, hostname=hostname, port=port, auth=auth)

# Air2
def callback_air2_cb(data):
    # rospy.loginfo(rospy.get_caller_id() + "Air2_cb %d", data.data)
    publish.single("air_iot/Air2_cb",data.data, hostname=hostname, port=port, auth=auth)

def callback_air2_alarm(data):
    # rospy.loginfo(rospy.get_caller_id() + "Air2_alarm %d", data.data)
    publish.single("air_iot/Air2_alarm",data.data, hostname=hostname, port=port, auth=auth)

def callback_air2_low_pressure(data):
    # rospy.loginfo(rospy.get_caller_id() + "Air2_low_pressure%d", data.data)
    publish.single("air_iot/Air2_low_pressure",data.data, hostname=hostname, port=port, auth=auth)

def callback_air2_high_pressure(data):
    # rospy.loginfo(rospy.get_caller_id() + "Air2_high_pressure %d", data.data)
    publish.single("air_iot/Air2_high_pressure",data.data, hostname=hostname, port=port, auth=auth)

 # air 1 2 status
def callback_air1_status(data):
    # rospy.loginfo(rospy.get_caller_id() + "Air1_status %d", data.data)
    publish.single("air_iot/Air1_status",data.data, hostname=hostname, port=port, auth=auth)

def callback_air2_status(data):
    # rospy.loginfo(rospy.get_caller_id() + "Air2_status %d", data.data)
    publish.single("air_iot/Air2_status",data.data, hostname=hostname, port=port, auth=auth)

# temp humid ros
def callback_temp_ros(data):
    # rospy.loginfo(rospy.get_caller_id() + "temp_ros %d", data.data)
    publish.single("air_iot/temp_ros",data.data, hostname=hostname, port=port, auth=auth)

def callback_humid_ros(data):
    # rospy.loginfo(rospy.get_caller_id() + "humid_ros %d", data.data)
    publish.single("air_iot/humid_ros",data.data, hostname=hostname, port=port, auth=auth)

# air 1 & 2 auto manual status
def callback_air1_auto_status(data):
    # rospy.loginfo(rospy.get_caller_id() + "Air1_auto_status %d", data.data)
    publish.single("air_iot/Air1_auto_status",data.data, hostname=hostname, port=port, auth=auth)

def callback_air1_manual_status(data):
    # rospy.loginfo(rospy.get_caller_id() + "Air1_manual_status %d", data.data)
    publish.single("air_iot/Air1_manual_status",data.data, hostname=hostname, port=port, auth=auth)

def callback_air2_auto_status(data):
    # rospy.loginfo(rospy.get_caller_id() + "Air2_auto_status %d", data.data)
    publish.single("air_iot/Air2_auto_status",data.data, hostname=hostname, port=port, auth=auth)

def callback_air2_manual_status(data):
    # rospy.loginfo(rospy.get_caller_id() + "Air2_manual_status %d", data.data)
    publish.single("air_iot/Air2_manual_status",data.data, hostname=hostname, port=port, auth=auth)

#setting
def callback_air_duty_time(data):
    rospy.loginfo(rospy.get_caller_id() + "Air_duty_time %d", data.data)
    publish.single("air_iot/Air_duty_time",data.data, hostname=hostname, port=port, auth=auth)
def callback_air_temp_on(data):
    rospy.loginfo(rospy.get_caller_id() + "Air_temp_on %d", data.data)
    publish.single("air_iot/Air_temp_on",data.data, hostname=hostname, port=port, auth=auth)
def callback_air_temp_off(data):
    rospy.loginfo(rospy.get_caller_id() + "Air_temp_off %d", data.data)
    publish.single("air_iot/Air_temp_off",data.data, hostname=hostname, port=port, auth=auth)
def callback_air_humid_on(data):
    rospy.loginfo(rospy.get_caller_id() + "Air_humid_on %d", data.data)
    publish.single("air_iot/Air_humid_on",data.data, hostname=hostname, port=port, auth=auth)
def callback_air_humid_off(data):
    rospy.loginfo(rospy.get_caller_id() + "Air_humid_off %d", data.data)
    publish.single("air_iot/Air_humid_off",data.data, hostname=hostname, port=port, auth=auth)
def callback_dcfan_tempon(data):
    rospy.loginfo(rospy.get_caller_id() + "Dcfan_tempon %d", data.data)
    publish.single("air_iot/Dcfan_tempon",data.data, hostname=hostname, port=port, auth=auth)
def callback_dcfan_step(data):
    rospy.loginfo(rospy.get_caller_id() + "dcfan_step %d", data.data)
    publish.single("air_iot/Dcfan_step",data.data, hostname=hostname, port=port, auth=auth)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('Air_IOT_PI4', anonymous=True)

    rospy.Subscriber("Air1_auto_status", Int8, callback_air1_auto_status)
    rospy.Subscriber("Air1_manual_status", Int8, callback_air1_manual_status)
    rospy.Subscriber("Air1_status", Int8, callback_air1_status)
    rospy.Subscriber("Air1_cb", Int8, callback_air1_cb)
    rospy.Subscriber("Air1_alarm", Int8, callback_air1_alarm)
    rospy.Subscriber("Air1_low_pressure", Int8, callback_air1_low_pressure)
    rospy.Subscriber("Air1_high_pressure", Int8, callback_air1_high_pressure)
   
    rospy.Subscriber("Air2_auto_status", Int8, callback_air2_auto_status)
    rospy.Subscriber("Air2_manual_status", Int8, callback_air2_manual_status)       
    rospy.Subscriber("Air2_status", Int8, callback_air2_status)
    rospy.Subscriber("Air2_cb", Int8, callback_air2_cb)
    rospy.Subscriber("Air2_alarm", Int8, callback_air2_alarm)
    rospy.Subscriber("Air2_low_pressure", Int8, callback_air2_low_pressure)
    rospy.Subscriber("Air2_high_pressure", Int8, callback_air2_high_pressure)
    
    rospy.Subscriber("temp_ros", Float32, callback_temp_ros)
    rospy.Subscriber("humid_ros", Float32, callback_humid_ros)
    
    #setting
    rospy.Subscriber("Air_duty_time", Int8, callback_air_duty_time)
    rospy.Subscriber("Air_temp_on", Int8, callback_air_temp_on)
    rospy.Subscriber("Air_temp_off", Int8, callback_air_temp_off)
    rospy.Subscriber("Air_humid_on", Int8, callback_air_humid_on)
    rospy.Subscriber("Air_humid_off", Int8, callback_air_humid_off)
    rospy.Subscriber("Dc_fan_temp_on", Int8, callback_dcfan_tempon)
    rospy.Subscriber("Dc_fan_step", Float32, callback_dcfan_step)

    # msg = subscribe.simple("air_iot/set_time", hostname=hostname)
    # print("%s %s" % (msg.topic, msg.payload))

    # spin() simply keeps python from exiting until this node is stopped
    
    rospy.spin()

if __name__ == '__main__':
    listener()
