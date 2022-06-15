#!/usr/bin/env python3

# Libraries
import rospy
import csv
import os
import sys
import time

# Messages
from current_sense_node.msg import motor_currents

data_dir = '/home/pi/orbweaver_ws/src/data_logging_node/data'
data_file = ''
generated_file = False

def data_callback(req):
    global data_dir, data_file, generated_file

    motor_data = req.motor_current
    num_motors = len(motor_data)

    if generated_file == False:
        flag = True
        counter = 0

        while flag == True:
            if os.path.exists(data_dir + '/data' + str(counter) + '.csv') == False:
                flag = False
                generated_file = True
                data_file = data_dir + '/data' + str(counter) + '.csv'
            else:
                counter += 1

        with open(data_file, 'w') as csv_file:
            csv_writer = csv.writer(csv_file)
            header_row = []
            for i in range(num_motors):
                header_row.append('Motor' + str(i + 1))
            csv_writer.writerow(header_row)
        print('Created data file and header row written')
    else:
        with open(data_file, 'a') as csv_file:
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(motor_data)




# Initializing the ROS Node and subscriber
rospy.init_node('data_logging_node', anonymous = False)
sub = rospy.Subscriber('/sensor/motor_current', motor_currents, data_callback)
r = rospy.Rate(100)

while not rospy.is_shutdown():
    continue