#!/usr/bin/env python

import rospy
import math

from sensor_msgs import msg
import serial
from imu_driver.msg import raw
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler


if __name__ == '__main__':
    rospy.init_node('IMU_driver')
    port = serial.Serial('/dev/ttyUSB0', 115200, timeout = 3.)
    rospy.logdebug("Starting Sensor")
    imu_pub = rospy.Publisher('IMU',Imu, queue_size=5)
    mag_pub = rospy.Publisher('MAG', MagneticField, queue_size=5)
    raw_pub = rospy.Publisher('RAW', raw, queue_size=5)
    try:
        while not rospy.is_shutdown():
            linesep = port.readline()
            extractor = linesep.split(',')
            datar = raw()
            datai = Imu()
            datam = MagneticField()
            datar.header = Header()
            datar.header.stamp = rospy.Time.now()
            datar.data = linesep
            raw_pub.publish(datar)
            #print(linesep)
            if extractor[0] == '':
                rospy.logwarn('No data')

            elif extractor[0].strip() == '$VNYMR':
                datai.header = Header()
                datai.header.stamp = rospy.Time.now()
                
                yaw_deg = extractor[1]
                pitch_deg = extractor[2]
                roll_deg = extractor[3]
                yaw_rad = (float(yaw_deg)* math.pi)/180.0
                pitch_rad = (float(pitch_deg)* math.pi)/180.0
                roll_rad = (float(roll_deg)* math.pi)/180.0
                q = quaternion_from_euler(yaw_rad, pitch_rad, roll_rad)
                
                datai.orientation.x = q[0]
                datai.orientation.y = q[1]
                datai.orientation.z = q[2]
                datai.orientation.w = q[3]

                datai.linear_acceleration.x = float(extractor[7])
                datai.linear_acceleration.y = float(extractor[8])
                datai.linear_acceleration.z = float(extractor[9])
                
                datai.angular_velocity.x = float(extractor[10])
                datai.angular_velocity.y = float(extractor[11])
                angular_z = extractor[12].split('*')
                datai.angular_velocity.z = float(angular_z[0])

                datam.header = Header()
                datam.header.stamp = rospy.Time.now()

                datam.magnetic_field.x = float(extractor[4])
                datam.magnetic_field.y = float(extractor[5])
                datam.magnetic_field.z = float(extractor[6])

                imu_pub.publish(datai)
                mag_pub.publish(datam)
    
    except rospy.ROSInterruptException:
        port.close()
    
    except serial.serialutil.SerialException:
        rospy.loginfo('Shutting down IMU')

