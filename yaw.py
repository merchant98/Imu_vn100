#!/usr/bin/python

from matplotlib import colors
import scipy.integrate as sp
import rosbag
import numpy as np
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt
import math
yaw = []
time = []
roll = []
pitch = []
accel_x = []
accel_y = []
accel_z = []
ang_z = []
mag_x = []
mag_y = []
mag_z = []
utm_north = []
utm_east = []
t_g = []


bag2 = rosbag.Bag('/home/sahil/Downloads/gps_data.bag')
for topic,gps,t in bag2.read_messages(topics=['/gps_data']):
    utm_north.append(gps.utm_northing)
    utm_east.append(gps.utm_easting)
    t_g.append(gps.header.stamp.secs)


utm_n = np.array(utm_north)
utm_e = np.array(utm_east)
Time_gps = np.array(t_g)


bag = rosbag.Bag('/home/sahil/catkin_ws/src/Analysis/trim_imu.bag')
for topic,Imu,t in bag.read_messages(topics=['/imu_data']):
    accel_x.append(Imu.linear_acceleration.x)
    accel_y.append(Imu.linear_acceleration.y)
    accel_z.append(Imu.linear_acceleration.z)
    xyz = [Imu.orientation.x, Imu.orientation.y, Imu.orientation.z, Imu.orientation.w]
    ang_z.append(Imu.angular_velocity.z)
    rpy = euler_from_quaternion(xyz)
    roll.append(rpy[0])
    pitch.append(rpy[1])
    yaw.append(rpy[2])
    time.append(Imu.header.stamp.secs)

for topic,MagneticField,t in bag.read_messages(topics=['/mag_data']):
    mag_x.append(MagneticField.magnetic_field.x)
    mag_y.append(MagneticField.magnetic_field.y)
    mag_z.append(MagneticField.magnetic_field.z)


magx = np.array(mag_x[0:10000])
magy = np.array(mag_y[0:10000])
magz = np.array(mag_z)

ang_rate = np.array(ang_z)
yaw_rate = np.array(yaw)
Time = np.array(time)
accelx = np.array(accel_x)
accely = np.array(accel_y)
accelz = np.array(accel_z)
yaw_angle = sp.cumtrapz(ang_rate, x=Time, initial =0)
yaw_angle2 = sp.cumtrapz(yaw_rate, x=Time, initial=0)
vel_x = sp.cumtrapz(accelx, x=Time, initial =0)
vel_y = sp.cumtrapz(accely, x=Time, initial =0)
vel_z = sp.cumtrapz(accelz, x=Time, initial =0)



#---------Part 2b-1---------------
#plt.figure('Yaw Integrated from Angular')
#plt.plot(Time, yaw_angle, '.')
#plt.figure('Angular raw')
#plt.plot(Time, ang_rate, '.')
#plt.figure('Yaw orientation')
#plt.plot(Time, yaw_angle2, '.')
plt.figure('Magx v Magy')
plt.grid(axis='both')
plt.plot(magx, magy, '.')

#--------Part 2b-2-----------
plt.figure('Accel')
plt.plot(Time, accel_x,'r')
plt.figure('error distribution')
plt.hist(accel_x-np.mean(accel_x), color='red', edgecolor='black')

'''
#Accelaration integrated
plt.figure('Velocity via Accel')
plt.plot(vel_x, vel_y,'black')
'''
#GPS data plot
magv = []
velo_n = np.diff(utm_n)
velo_e = np.diff(utm_e)
td = np.diff(Time_gps)
print(Time[250])
north = velo_n/td
east = velo_e/td
print(Time_gps[103])
plt.figure('GPS')
plt.plot(utm_e[0:103], utm_n[0:103], '.')
#plt.figure('GPS 2')
#plt.plot(Time_gps, utm_e, 'b')
#plt.plot(Time_gps,utm_e, '.')
#plt.figure('Velocity via GPS')
#plt.plot(Time_gps[1:], north ,'r')
#plt.plot(Time_gps[1:],east, 'b')

plt.show()