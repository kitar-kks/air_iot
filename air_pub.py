#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Int8
import paho.mqtt.subscribe as subscribe

hostname = "34.139.76.224"
port = 1883
auth = {
 'username':'admin',
 'password':'p@ssw0rd'
}
mqtt_topic = [("air_iot/set_time",0) ,("air_iot/set_temp_on",0) ,("air_iot/set_temp_off",0) ,("air_iot/set_humid_on",0) ,("air_iot/set_humid_on",0)]

def talker():
    Rset_time = rospy.Publisher('set_time', Int8, queue_size=10)
    Rset_temp_on = rospy.Publisher('set_temp_on', Int8, queue_size=10)
    Rset_temp_off = rospy.Publisher('set_temp_off', Int8, queue_size=10)
    Rset_humid_on = rospy.Publisher('set_humid_on', Int8, queue_size=10)
    Rset_humid_off = rospy.Publisher('set_humid_off', Int8, queue_size=10)

    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = subscribe.simple(mqtt_topic, hostname=hostname,auth=auth)
        if(msg.topic == 'air_iot/set_time'):
            time_for_arduino = msg.payload
        if(msg.topic == 'air_iot/set_temp_on'):
            set_temp_on = msg.payload       
        # temp_on = subscribe.simple("air_iot/set_temp_on", hostname=hostname ,auth=auth)
        # temp_off = subscribe.simple("air_iot/set_temp_off", hostname=hostname ,auth=auth)
        # humid_on = subscribe.simple("air_iot/set_humid_on",hostname=hostname,auth=auth)
        # humid_off = subscribe.simple("air_iot/set_humid_on",hostname=hostname,auth=auth)

        # time_for_arduino = int(msg.payload,10)
        # set_temp_on = int(temp_on.payload,10)
        # set_temp_off = int(temp_off.payload,10)
        # set_humid_on = int(humid_on.payload,10)
        # set_humid_off = int(humid_off.payload,10)
        
        rospy.loginfo(time_for_arduino)
        Rset_time.publish(time_for_arduino)
        Rset_temp_on.publish(set_temp_on)
        # Rset_temp_off.publish(set_temp_off)
        # Rset_humid_on.publish(set_humid_on)
        # Rset_humid_off.publish(set_humid_off)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass