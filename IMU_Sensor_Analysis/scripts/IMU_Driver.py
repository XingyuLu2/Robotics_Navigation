#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import utm
import numpy as np

from sensor_msgs.msg import Imu, MagneticField

def cov_to_quat(yaw, pitch, roll):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]

if __name__ == '__main__':
    #data_example = "$GPGGA,134658.00,5106.9792,N,11402.3003,W,2,09,1.0,1048.47,M,-16.27,M,08,AAAA*60"
    
    SENSOR_NAME = "imu_sensor"  
    # confgure the Port parameters
    rospy.init_node('data_imu_sensor')
    serial_port = rospy.get_param('~port','/dev/ttyUSB0')
    serial_baud = rospy.get_param('~baudrate',115200)
    # initialize the port
    port = serial.Serial(serial_port, serial_baud)

    imu_pub = rospy.Publisher(SENSOR_NAME+'/imu', Imu, queue_size = 10)
    mag_pub = rospy.Publisher(SENSOR_NAME+'/mag', MagneticField, queue_size = 10)
    try:
        while not rospy.is_shutdown():
            imu_data = port.readline()
	    # get the $GPGGA format data message
	    if imu_data[0:6] == "$VNYMR":
	        # get the data we want bu cutting the header and check num
	        # and check whether it is complete data
                if len(imu_data) < 122:
	            print('No complete data collected !')
		    continue
                print(imu_data) 
	        imu_data = imu_data[7:117]
	        split_data = imu_data.split(",")

	        imu_msg = Imu()
	        mag_msg = MagneticField()
	        # magnetometer data: split_data[4:6]
	        MagX = float(split_data[3])
	        MagY = float(split_data[4])
	        MagZ = float(split_data[5])
	        Mag_vector = [MagX, MagY, MagZ]

	        # Convert the Yaw/Pitch/Roll data ino Quaternion
	        yaw = float(split_data[0]) * 3.1415926 / 180
	        pitch = float(split_data[1]) * 3.1415926 / 180
	        roll = float(split_data[2]) * 3.1415926 / 180
	        quater = cov_to_quat(yaw, pitch, roll)

     	        # accelerator data: split_data[7:9]
	        AccX = float(split_data[6])
	        AccY = float(split_data[7])
	        AccZ = float(split_data[8])

	        # Gyro(angular rates) data: split_data[10:12]
	        GyroX = float(split_data[9])
	        GyroY = float(split_data[10])
	        GyroZ = float(split_data[11])

	        # Give data to Imu message
	        imu_msg.header.stamp = rospy.Time.now()
	        imu_msg.orientation.x = quater[0]
	        imu_msg.orientation.y = quater[1]
	        imu_msg.orientation.z = quater[2]
	        imu_msg.orientation.w = quater[3]

	        imu_msg.linear_acceleration.x = AccX
	        imu_msg.linear_acceleration.y = AccY
	        imu_msg.linear_acceleration.z = AccZ

	        imu_msg.angular_velocity.x = GyroX
	        imu_msg.angular_velocity.y = GyroY
	        imu_msg.angular_velocity.z = GyroZ	

	        # Give data to MagneticField Messages
	        mag_msg.magnetic_field.x = MagX
	        mag_msg.magnetic_field.y = MagY
	        mag_msg.magnetic_field.z = MagZ
	    
	        # Publish two messages
	        imu_pub.publish(imu_msg)
	        mag_pub.publish(mag_msg)
            rospy.sleep(0.01)

    except rospy.ROSInterruptException:
        port.close()
    
    except serial.serialutil.SerialException:
        rospy.loginfo("Shutting down paro_depth node...")
