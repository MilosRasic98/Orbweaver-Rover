#!/usr/bin/env python3

# Libraries
import rospy
import smbus
import time
import crc8

# Messages
from current_sense_node.msg import motor_currents
from std_msgs.msg import Float32

# Services
from current_sense_node.srv import max40080_service

# Parameters
NUM_MOTORS  = 1
AVG_SIZE    = 10

#max40080_addr = 0x21
max40080_addr = [0x21, 0x21, 0x21, 0x21, 0x21, 0x21]

bus = smbus.SMBus(1)

adc_sample_rate = [15, 99999999, 23.45, 30, 37.5, 47.1, 60, 93.5, 120, 150, 234.5, 375, 468.5, 750, 1000]
digital_filter  = [0, 8, 16, 32, 64, 128]
range_multiplier = 1.25

def dec_to_binary(num):
    bin_num = str(bin(num).replace('0b',''))
    while True:
        if len(bin_num) < 8:
            bin_num = '0' + bin_num
        else:
            return bin_num

def convert_to_ma(reading):

    # Getting the needed bytes from the reading
    first_byte_dec  = reading[0]
    second_byte_dec = reading[1]

    # Converting them to binary
    first_byte_bin  = dec_to_binary(first_byte_dec)
    second_byte_bin = dec_to_binary(second_byte_dec)

    # Combining them into the register value
    register_value  = second_byte_bin + first_byte_bin

    # Extracting the data from the register value
    data_valid_flag = register_value[0]
    sign            = register_value[3]
    reg_val         = register_value[4:]

    if data_valid_flag != '1':
        # If this is the case, we have a bad reading
        print('Error reading from sensor')
        return -10000.00

    # Value of this multiplier depends on how the current flows
    multiplier = 1

    # If the value of sign is '1' that means that the current flow is negative
    if sign == '1':
        multiplier = -1
        cur_mag = 4096 - int(reg_val, 2)
    else:
        cur_mag = int(reg_val, 2)

    # Now we need to convert that data into mA
    global range_multiplier

    current_value = multiplier * 1000 * cur_mag * range_multiplier / (4095 * 25  * 0.01)

    # We return the converted value
    return current_value

def calc_crc8(max_address, config_reg, LSB, MSB):
    new_hash = crc8.crc8()
    new_hash.update(bytes.fromhex(str(format(2 * max_address, '#04x'))[2:]))
    new_hash.update(bytes.fromhex(str(format(config_reg, '#04x'))[2:]))
    new_hash.update(bytes.fromhex(str(format(LSB, '#04x'))[2:]))
    new_hash.update(bytes.fromhex(str(format(MSB, '#04x'))[2:]))
    return new_hash.hexdigest()

# Function for getting the adc sample rate code
def adc_code(sample_rate):
    # Loading our list of available frequencies
    global adc_sample_rate
    # Checking if we can find the frequency in our list, if not, we return an error
    try:
        adc_sample_rate.index(sample_rate)
    except:
        print('Error')
        return False, '2222'
    # Returning the value as a 4 digit binary code that we need for configuring the MAX40080
    return True, format(adc_sample_rate.index(sample_rate), '04b')

# Function for getting the digital filter code
def filter_code(filter):
    # Loading our list of available filters
    global digital_filter
    # Checking if we can find the filter in our list, if not, we return an error
    try:
        digital_filter.index(filter)
    except:
        print('Error')
        return False, '2222'
    # Returning the value as a 3 digit binary code that we need for configuring the MAX40080
    return True, format(digital_filter.index(filter), '03b')

# Function for getting the current range bit
def range_code(range):
    # Checking if the user selected the right range
    if range != 5 and range != 1:
        print('Error')
        return False, '2'
    # We also need to load and update the multiplier
    global range_multiplier
    # If the user selected the range of 5A
    if range == 5:
        range_multiplier = 1.25
        return True, '0'
    # If the user selected the range of 1A
    if range == 1:
        range_multiplier = 0.25
        return True, '1'

# Function for generating the LSB and MSB for updating the configuration of the MAX40080
def configuration_bytes(filter, adc, range):
    # Generating the codes that we need for the I2C message
    flag1, filter_val    = filter_code(filter)
    flag2, adc_val       = adc_code(adc)
    flag3, range_val     = range_code(range) 
    # Checking if any of flags are Flase
    if flag1 == False or flag2 == False or flag3 == False:
        return 0x00, 0x00, False
    # Generating the LSB & MSB
    LSB                 = '0' + range_val + '100011'
    MSB                 = '0' + filter_val + adc_val
    # After generating the values, we need to convert them to hex
    LSB_hex             = int(LSB, 2)
    MSB_hex             = int(MSB, 2)
    # Now we return the values
    return MSB_hex, LSB_hex, True

# Function for reading the current configuration of the MAX40080
def current_config(max_address):
    # Reading the register value
    reg_data            = bus.read_i2c_block_data(max_address, 0x00, 3)
    # Extracting the LSB & MSB
    LSB                 = reg_data[0]
    MSB                 = reg_data[1]
    # Generating the register value from LSB & MSB
    reg_value           = str(format(MSB, '08b')) + str(format(LSB, '08b'))
    # Loading the data arrays
    global adc_sample_rate, digital_filter, range_multiplier
    # Reading the data
    filter_val          = digital_filter[int(reg_value[1:4], 2)] 
    adc_val             = adc_sample_rate[int(reg_value[4:8], 2)]
    range               = 5
    range_multiplier    = 1.25

    if reg_value[9] == '1':
        range = 1
        range_multiplier = 0.25

    return filter_val, adc_val, range

# Function for changing the configuration of a single MAX40080
def configure_max40080(max_address, filter, adc, range):
    MSB, LSB, flag = configuration_bytes(filter, adc, range)
    if flag == True:
        print('MSB: ' + hex(MSB) + '(' + str(MSB) + ')')
        print('LSB: ' + hex(LSB) + '(' + str(LSB) + ')')
        CRCB = calc_crc8(max_address, 0x00, LSB, MSB)
        print('CRC8: 0x', CRCB) 
        # Updating the configuration
        bus.write_i2c_block_data(max_address, 0x00, [LSB, MSB, int('0x' + CRCB, base = 16)])
        time.sleep(0.1)
        print(current_config(max_address))
        return True
    return False

# Service callback function
def max_srv_callback(req):
    # Loading up the number of sensors and their addresses
    global max40080_addr, NUM_MOTORS
    # First checking what kind of command we're dealing with
    if req.command == 'read':
        # This is the case where we only print out the data
        if req.sensor > NUM_MOTORS:
            # This is an error because we requested info about a device that isn't activated
            print('This device is out of range for the defined number of sensors')
            return False, 0, 0, 0
        # This is the case where we go through all of the devices and print the data out for all of them
        if req.sensor == 0:
            print('Number of sensors configured: ', NUM_MOTORS)
            for i in range(NUM_MOTORS):
                print('Sensor ' + str(i + 1) + ' configuration:')
                flt, adc, rng = current_config(max40080_addr[i])
                print('Filter:  ' + str(flt))
                print('ADC:     ' + str(adc) + 'ksps')
                print('Range:   ' + str(rng) + 'A')
            return True, 0, 0, 0
        # This is the case where we ask for the data of only one sensor
        print('Sensor ' + str(req.sensor) + ' configuration:')
        flt, adc, rng = current_config(max40080_addr[req.sensor + 1])
        print('Filter:  ' + str(flt))
        print('ADC:     ' + str(adc) + 'ksps')
        print('Range:   ' + str(rng) + 'A')
        return True, flt, adc, rng
    # This is the case where we want to configure a sensor or all at the same time
    elif req.command == 'config' or req.command == 'configure':
        # In this case, we want to configure all of the sensors at once
        if req.sensor == 0:
            for i in range(NUM_MOTORS):
                # First we need to read the old configuration
                flt, adc, rng = current_config(max40080_addr[i])
                # Now we can work on updating the configuration
                # We check for every param if it's -1, because if it is, we keep the old config
                if req.param1 != -1:
                    flt = req.param1
                if req.param2 != -1:
                    adc = req.param2
                if req.param3 != -1:
                    rng = req.param3
                # Now that we have the latest param we want, we can update the sensor
                resp = configure_max40080(max40080_addr[i], flt, adc, rng)
                # If the response is False, we stop trying to update configuration
                if resp == False:
                    print('Error trying to update the configuration of device ', i + 1)
                    return False, 0, 0, 0
            # If we've gotten out of the loop successfully, the update was successfull
            return True, 0, 0, 0

        # This is the case where we update only a single sensor
        # Before we get to the sensor we need to check if the sensor is in range
        if req.sensor > NUM_MOTORS:
            print('Sensor out of range of defined sensors')
            return False, 0, 0, 0
        # First we need to read the old configuration
        flt, adc, rng = current_config(max40080_addr[req.sensor])
        # Now we can work on updating the configuration
        # We check for every param if it's -1, because if it is, we keep the old config
        if req.param1 != 255:
            flt = req.param1
        if req.param2 != 255:
            adc = req.param2
        if req.param3 != 255:
            rng = req.param3
        # Now that we have the latest param we want, we can update the sensor
        resp = configure_max40080(max40080_addr[req.sensor], flt, adc, rng)
        # If the response is False, we stop trying to update configuration
        if resp == False:
            print('Error trying to update the configuration of device ', req.sensor)
            return False, 0, 0, 0
        # Now we can check the config and return the latest values
        flt, adc, rng = current_config(max40080_addr[req.sensor])
        return True, flt, adc, rng

# Node startup configuration
print('Starting configuration of MAX40080')
for i in range(NUM_MOTORS):
    flag = configure_max40080(max40080_addr[i], 128, 120, 5)
    if flag == True:
        time.sleep(0.1)
        flt, adc, rng = current_config(max40080_addr[i])
        print('Filter: ' + str(flt))
        print('ADC Sample Rate: ' + str(adc) + 'ksps')
        print('Current range: ' + str(rng) + 'A')
print('Configuration done')


# Initializing the ROS Node and publisher
rospy.init_node('current_sense_node', anonymous = False)
pub = rospy.Publisher('/sensor/motor_current', motor_currents, queue_size=10)
max_srv = rospy.Service('/sensor/max40080', max40080_service, max_srv_callback)
r = rospy.Rate(100)

# This is the main loop of our node
while not rospy.is_shutdown():
    pub_msg = []
    for i in range(NUM_MOTORS):
        milliamps = 0.00
        for j in range(AVG_SIZE):
            reading = bus.read_i2c_block_data(max40080_addr[0], 0x0C, 3)
            milliamps += convert_to_ma(reading)
            time.sleep(0.001)
        pub_msg.append(milliamps / 10)
    pub.publish(pub_msg)
    r.sleep()
