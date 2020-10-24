#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import utm
from LAB3.msg import rtk_msg

def conv(data, num):
    # from DDDMM.MMMMM type to DD.DDDDDD type
    seg = data.split(".")
    head = seg[0]
    tail =  seg[1]
    deg = head[0:num]
    mini = head[num:]
    results = float(deg) + (float(mini) / 60) + ((float(tail)/ 10000000) / 60)
    return results

if __name__ == '__main__':
    # data_example = "$GNGGA,223249.20,4220.2350863,N,07105.2183547,W,5,09,1.2,-0.020,M,-28.727,M,0.6,*42"
    
    SENSOR_NAME = "gps_sensor"  
    # confgure the Port parameters
    rospy.init_node('data_gps_sensor')
    serial_port = rospy.get_param('~port','/dev/ttyACM0')
    serial_baud = rospy.get_param('~baudrate',115200)
    # initialize the port
    port = serial.Serial(serial_port, serial_baud)

    gps_pub = rospy.Publisher(SENSOR_NAME+'/rtk', rtk_msg, queue_size = 10)

    while not rospy.is_shutdown():
        gps_data = port.readline()
        # gps_data = data_example
	# get the $GPGGA format data message
	if gps_data[0:6] == "$GNGGA":
	    # get the Time, Latitude, Longtitude, Altitude
	    print(gps_data)
	    gps_message = rtk_msg()
	    split_data = gps_data.split(",")
	    latitude = conv(split_data[2],2)
	    longtitude = conv(split_data[4],3)
	    altitude = float(split_data[9])
	    if(split_data[3] == 'S'):
                latitude = -latitude
	    if(split_data[5] == 'W'):
		longtitude = -longtitude
	    Fix_Quality = int(split_data[6])

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
	    gps_message.FixQuality = Fix_Quality
	    print(gps_message)
	    # Publish the message
	    gps_pub.publish(gps_message)
	# rospy.sleep(0.5)









