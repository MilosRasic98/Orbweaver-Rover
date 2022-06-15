#!/usr/bin/env python3

import rospy
import serial
from std_msgs.msg import Float32

tt_arduino = serial.Serial("/dev/ttyUSB0", 9600)

rospy.init_node('torque_test_stand', anonymous = False)
pub = rospy.Publisher('/test_equipment/measured_torque', Float32, queue_size=10)
r = rospy.Rate(10)

print('Torque Test Stand Node Started!')

while not rospy.is_shutdown():
    raw_data        = str(tt_arduino.readline())
    extracted_data  = raw_data[2:raw_data.find('\\r')]
    converted_data  = float(extracted_data)
    pub.publish(converted_data)
    r.sleep()