#!/usr/bin/env python3

from socket import timeout
import rospy
import serial
import time

# Messages
from current_sense_node.msg import motor_currents

# Services
from motor_controller.srv import driver_msg

set_current = 0 # mA
current_reading = 0 # mA
new_reading = False
last_reading = -1
kp = 0.008
ki = 0.001
e  = 0
e_sum = 0
max_val = 1023
min_val = 0
power = 0
current_threshold = 15 #mA

motor_controller = serial.Serial("/dev/ttyACM0", 9600, timeout = 0.1)

# Function for the subscriber
def data_callback(req):
    global current_reading, new_reading
    current_reading = req.motor_current[0]
    new_reading = True

# Function for sending the duty cycle to the motor_controller
def send_command(dc):
    global motor_controller
    msg = str(dc) + '\n'
    motor_controller.write(msg.encode('utf-8'))

# Function for the service callback
def srv_callback(req):
    global set_current
    if req.command == 'cc':
        set_current = req.value
    else:
        return False
    return True
    

# Initializing the ROS Node and subscriber
rospy.init_node('motor_controller', anonymous = False)
sub = rospy.Subscriber('/sensor/motor_current', motor_currents, data_callback)
srv = rospy.Service('/control/motor', driver_msg, srv_callback)
r = rospy.Rate(100)
send_command(0)
time.sleep(1)

while not rospy.is_shutdown():

    if new_reading == True:
        new_reading = False
        e = set_current - current_reading
        if abs(e) > current_threshold: 
            e_sum += e
            power = e * kp + e_sum * ki

            if power > max_val:
                e_sum -= e
                power = max_val

            if power < min_val:
                e_sum -= e
                power = min_val

            print(power)
            send_command(int(power))
    
    r.sleep()