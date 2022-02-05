#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Int8
from std_msgs.msg import Float32
import paho.mqtt.subscribe as subscribe

hostname = "34.139.76.224"
port = 1883
auth = {
 'username':'admin',
 'password':'p@ssw0rd'
}
mqtt_topic = [("air_iot/set_time",0) ,("air_iot/set_temp_on",0) ,("air_iot/set_temp_off",0) ,("air_iot/set_humid_on",0) ,
("air_iot/set_humid_off",0),("air_iot/set_dcfan_temp_on",0),("air_iot/set_dcfan_step",0)]

def talker():
    Rset_time = rospy.Publisher('set_time', Int8, queue_size=10)
    Rset_temp_on = rospy.Publisher('set_temp_on', Int8, queue_size=10)
    Rset_temp_off = rospy.Publisher('set_temp_off', Int8, queue_size=10)
    Rset_humid_on = rospy.Publisher('set_humid_on', Int8, queue_size=10)
    Rset_humid_off = rospy.Publisher('set_humid_off', Int8, queue_size=10)
    Rset_dcfan_temp_on = rospy.Publisher('set_dcfan_temp_on', Int8, queue_size=10)
    Rset_dcfan_step = rospy.Publisher('set_dcfan_step', Float32, queue_size=10)

    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = subscribe.simple(mqtt_topic, hostname=hostname,auth=auth)
        if(msg.topic == 'air_iot/set_time'):
            time_for_arduino = int(msg.payload,10)
            # rospy.loginfo(time_for_arduino)
            Rset_time.publish(time_for_arduino) 

        if(msg.topic == 'air_iot/set_temp_on'):
            set_temp_on = int(msg.payload,10)
            Rset_temp_on.publish(set_temp_on)  

        if(msg.topic == 'air_iot/set_temp_off'):
            set_temp_off = int(msg.payload,10)
            Rset_temp_off.publish(set_temp_off) 

        if(msg.topic == 'air_iot/set_humid_on'):
            set_humid_on = int(msg.payload,10)
            Rset_humid_on.publish(set_humid_on)  

        if(msg.topic == 'air_iot/set_humid_off'):
            set_humid_off = int(msg.payload,10)
            Rset_humid_off.publish(set_humid_off)     
       
        if(msg.topic == 'air_iot/set_dcfan_temp_on'):
            set_dcfan_temp = int(msg.payload,10)
            Rset_dcfan_temp_on.publish(set_dcfan_temp)     
        
        if(msg.topic == 'air_iot/set_dcfan_step'):
            set_dcfan_step = float(msg.payload)
            rospy.loginfo(set_dcfan_step)
            Rset_dcfan_step.publish(set_dcfan_step)     
        # rospy.loginfo(time_for_arduino)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass