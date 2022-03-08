#!/usr/bin/env python
import paho.mqtt.publish as publish
import rospy
from std_msgs.msg import Int8
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int8MultiArray
from std_msgs.msg import UInt32MultiArray
from std_msgs.msg import Int32MultiArray
hostname = "34.139.76.224"
port = 1883
auth = {
 'username':'admin',
 'password':'p@ssw0rd'
}


# Air1
def callback_air1_status(data):
    publish.single("air_iot/Air1_status",data.data, hostname=hostname, port=port, auth=auth)
def callback_air1_auto_status(data):
    publish.single("air_iot/Air1_auto_status",data.data, hostname=hostname, port=port, auth=auth)
def callback_air1_manual_status(data):
    publish.single("air_iot/Air1_manual_status",data.data, hostname=hostname, port=port, auth=auth)
def callback_air1_alarm(data):
    publish.single("air_iot/Air1_alarm",data.data, hostname=hostname, port=port, auth=auth)
def callback_air1_cb(data):
    publish.single("air_iot/Air1_cb",data.data, hostname=hostname, port=port, auth=auth)
def callback_air1_low(data):
    publish.single("air_iot/Air1_low_pressure",data.data, hostname=hostname, port=port, auth=auth)
def callback_air1_high(data):
    publish.single("air_iot/Air1_high_pressure",data.data, hostname=hostname, port=port, auth=auth)

# Air2
def callback_air2_status(data):
    publish.single("air_iot/Air2_status",data.data, hostname=hostname, port=port, auth=auth)
def callback_air2_auto_status(data):
    publish.single("air_iot/Air2_auto_status",data.data, hostname=hostname, port=port, auth=auth)
def callback_air2_manual_status(data):
    publish.single("air_iot/Air2_manual_status",data.data, hostname=hostname, port=port, auth=auth)
def callback_air2_alarm(data):
    publish.single("air_iot/Air2_alarm",data.data, hostname=hostname, port=port, auth=auth)
def callback_air2_cb(data):
    publish.single("air_iot/Air2_cb",data.data, hostname=hostname, port=port, auth=auth)
def callback_air2_low(data):
    publish.single("air_iot/Air2_low_pressure",data.data, hostname=hostname, port=port, auth=auth)
def callback_air2_high(data):
    publish.single("air_iot/Air2_high_pressure",data.data, hostname=hostname, port=port, auth=auth)

# temp humid ros
def callback_temp_ros(data):
    # rospy.loginfo(rospy.get_caller_id() + "temp_ros %d", data.data)
    publish.single("air_iot/temp_ros",data.data, hostname=hostname, port=port, auth=auth)
def callback_humid_ros(data):
    # rospy.loginfo(rospy.get_caller_id() + "humid_ros %d", data.data)
    publish.single("air_iot/humid_ros",data.data, hostname=hostname, port=port, auth=auth)

#setting
def callback_air_setting(data):
    # rospy.loginfo(rospy.get_caller_id() + "Air_duty_time %d", data.data)
    publish.single("air_iot/Air_duty_time",data.data[0], hostname=hostname, port=port, auth=auth)
    publish.single("air_iot/Air_temp_on",data.data[1], hostname=hostname, port=port, auth=auth)
    publish.single("air_iot/Air_temp_off",data.data[2], hostname=hostname, port=port, auth=auth)
    publish.single("air_iot/Air_humid_on",data.data[3], hostname=hostname, port=port, auth=auth)
    publish.single("air_iot/Air_humid_off",data.data[4], hostname=hostname, port=port, auth=auth)

def callback_dcfan_tempon(data):
    # rospy.loginfo(rospy.get_caller_id() + "Dcfan_tempon %d", data.data)
    publish.single("air_iot/Dcfan_tempon",data.data, hostname=hostname, port=port, auth=auth)
def callback_dcfan_step(data):
    # rospy.loginfo(rospy.get_caller_id() + "dcfan_step %d", data.data)
    publish.single("air_iot/Dcfan_step",data.data, hostname=hostname, port=port, auth=auth)

def callback_source_fail (data):
    # rospy.loginfo(rospy.get_caller_id() + "ac_fail %d", data.data[1])
    # rospy.loginfo(rospy.get_caller_id() + "dc_fail %d", data.data[2])
    publish.single("air_iot/ac_source_fail",data.data[0], hostname=hostname, port=port, auth=auth)
    publish.single("air_iot/dc_source_fail",data.data[1], hostname=hostname, port=port, auth=auth)

def callback_dc_fan(data):
    # rospy.loginfo("dc_fan1 %d", data.data[1])
    # rospy.loginfo(rospy.get_caller_id() + "dc_fan2 %d", data.data[2])
    publish.single("air_iot/dc_fan1",data.data[0], hostname=hostname, port=port, auth=auth)
    publish.single("air_iot/dc_fan2",data.data[1], hostname=hostname, port=port, auth=auth)

#power meter
def callback_voltage(data):
    publish.single("air_iot/v_system",data.data[0], hostname=hostname, port=port, auth=auth)
    publish.single("air_iot/v_p1",data.data[1], hostname=hostname, port=port, auth=auth)
    publish.single("air_iot/v_p2",data.data[2], hostname=hostname, port=port, auth=auth)
    publish.single("air_iot/v_p3",data.data[3], hostname=hostname, port=port, auth=auth)
    publish.single("air_iot/v_l1",data.data[4], hostname=hostname, port=port, auth=auth)
    publish.single("air_iot/v_l2",data.data[5], hostname=hostname, port=port, auth=auth)
    publish.single("air_iot/v_l3",data.data[6], hostname=hostname, port=port, auth=auth)
def callback_current(data):
    publish.single("air_iot/i_system",data.data[0], hostname=hostname, port=port, auth=auth)
    publish.single("air_iot/i_l1",data.data[1], hostname=hostname, port=port, auth=auth)
    publish.single("air_iot/i_l2",data.data[2], hostname=hostname, port=port, auth=auth)
    publish.single("air_iot/i_l3",data.data[3], hostname=hostname, port=port, auth=auth)
def callback_power(data):
    publish.single("air_iot/pf_3p",data.data[0], hostname=hostname, port=port, auth=auth)
    publish.single("air_iot/cp_3p",data.data[1], hostname=hostname, port=port, auth=auth)
def callback_enerygy(data):
    publish.single("air_iot/ac_power_3p",data.data[0], hostname=hostname, port=port, auth=auth)
    publish.single("air_iot/ac_energy",data.data[1], hostname=hostname, port=port, auth=auth)
    publish.single("air_iot/freq",data.data[2], hostname=hostname, port=port, auth=auth)
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('Air_IOT_PI4', anonymous=True)

    rospy.Subscriber("Air1_status", Int8, callback_air1_status)
    rospy.Subscriber("Air1_auto_status", Int8, callback_air1_auto_status)
    rospy.Subscriber("Air1_manual_status", Int8, callback_air1_manual_status)
    rospy.Subscriber("Air1_alarm", Int8, callback_air1_alarm)
    rospy.Subscriber("Air1_cb", Int8, callback_air1_cb)
    rospy.Subscriber("Air1_low_pressure", Int8, callback_air1_low)
    rospy.Subscriber("Air1_high_pressure", Int8, callback_air1_high)

    rospy.Subscriber("Air2_status", Int8, callback_air2_status)
    rospy.Subscriber("Air2_auto_status", Int8, callback_air2_auto_status)
    rospy.Subscriber("Air2_manual_status", Int8, callback_air2_manual_status)
    rospy.Subscriber("Air2_alarm", Int8, callback_air2_alarm)
    rospy.Subscriber("Air2_cb", Int8, callback_air2_cb)
    rospy.Subscriber("Air2_low_pressure", Int8, callback_air2_low)
    rospy.Subscriber("Air2_high_pressure", Int8, callback_air2_high)

    rospy.Subscriber("temp_ros", Float32, callback_temp_ros)
    rospy.Subscriber("humid_ros", Float32, callback_humid_ros)

    rospy.Subscriber("source_fail", Int8MultiArray, callback_source_fail)

    rospy.Subscriber("dc_fan_status", Int8MultiArray, callback_dc_fan)
    
    rospy.Subscriber("voltage", UInt32MultiArray, callback_voltage)
    rospy.Subscriber("current", UInt32MultiArray, callback_current)
    rospy.Subscriber("power", Int32MultiArray, callback_power)
    rospy.Subscriber("energy", UInt32MultiArray, callback_enerygy)

    #setting
    rospy.Subscriber("Air_setting", Int8MultiArray, callback_air_setting)
    rospy.Subscriber("Dc_fan_temp_on", Int8, callback_dcfan_tempon)
    rospy.Subscriber("Dc_fan_step", Float32, callback_dcfan_step)

    # msg = subscribe.simple("air_iot/set_time", hostname=hostname)
    # print("%s %s" % (msg.topic, msg.payload))

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
