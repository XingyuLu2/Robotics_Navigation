#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import utm
from Lab_Exercise.msg import gps_msg

def conv(data, num):
    # from DDDMM.MMMMM type to DD.DDDDDD type
    seg = data.split(".")
    head = seg[0]
    tail =  seg[1]
    deg = head[0:num]
    mini = head[num:]
    results = float(deg) + (float(mini) / 60) + ((float(tail)/ 10000) / 60)
    return results

if __name__ == '__main__':
    #data_example = "$GPGGA,134658.00,5106.9792,N,11402.3003,W,2,09,1.0,1048.47,M,-16.27,M,08,AAAA*60"
    
    SENSOR_NAME = "gps_sensor"  
    # confgure the Port parameters
    rospy.init_node('data_gps_sensor')
    serial_port = rospy.get_param('~port','/dev/ttyUSB0')
    serial_baud = rospy.get_param('~baudrate',4800)
    # initialize the port
    port = serial.Serial(serial_port, serial_baud)

    gps_pub = rospy.Publisher(SENSOR_NAME+'/gps', gps_msg, queue_size = 10)

    while not rospy.is_shutdown():
        gps_data = port.readline()
	# get the $GPGGA format data message
	if gps_data[0:6] == "$GPGGA":
	    # get the Time, Latitude, Longtitude, Altitude
	    print(gps_data)
	    gps_message = gps_msg()
	    split_data = gps_data.split(",")
	    latitude = conv(split_data[2],2)
	    longtitude = conv(split_data[4],3)
	    altitude = float(split_data[9])
	    if(split_data[3] == 'S'):
                latitude = -latitude
	    if(split_data[5] == 'W'):
		longtitude = -longtitude

	    # the data into UTM format
	    lat = float(latitude)
	    lot = float(longtitude)
	    utm_msg = utm.from_latlon(lat, lot)
	    utm_easting = utm_msg[0]
	    utm_northing = utm_msg[1]
	    utm_zone_num = utm_msg[2]
	    utm_zone_letter = utm_msg[3]
            # give the values to the messages
	    gps_message.header.stamp = rospy.Time.now()	    	    

	    gps_message.latitude = latitude;
	    gps_message.Longtitude = longtitude
	    gps_message.Altitude = altitude
	    gps_message.utm_easting = utm_easting
	    gps_message.utm_northing = utm_northing
	    gps_message.Zone = utm_zone_num
	    gps_message.letter = utm_zone_letter
	    # Publish the message
	    gps_pub.publish(gps_message)
	rospy.sleep(1.0) # 1.0









