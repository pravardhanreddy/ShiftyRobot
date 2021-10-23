#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import Bool
from time import sleep
from random import randint
from math import atan2,sin,cos,sqrt
from tf.transformations import euler_from_quaternion as efq

yaw,x,y,vx = 0,0,0,0
vel = Twist()

v_max = 0.4
v_inc = 0.01

xg = 5
yg = 0

obstacle = False

def odomCallback(odom):
	global yaw,x,y,vx
	quat = odom.pose.pose.orientation
	(roll,pitch,yaw) = efq([quat.x,quat.y,quat.z,quat.w])
	x = odom.pose.pose.position.x
	y = odom.pose.pose.position.y
	vx = odom.twist.twist.linear.x

def scanCallback(scan):
	global obstacle
	min_dist = min((scan.ranges[i] for i in range(-30, 30)))
	obstacle = any((scan.ranges[i] < 0.5 for i in range(-30, 30)))
	print(obstacle, min_dist)

error = 0
prev = 0
integral = 0
dt = 0.1
kp, ki, kd = 0.8, 0.1, 0.03
odom_reset = PoseWithCovarianceStamped()

def pid(target, current):
	global error, prev, integral, dt, kp, ki, kd
	error = target - current
	integral = integral + error * dt
	derivative = (error - prev)/dt
	prev = error
	return kp * error + ki * integral + kd * derivative;
    


def main():

	global vel, v_max, x, y, yaw, vx, xg, yg, obstacle
	rospy.loginfo("Enter main")

	vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	odom_pub = rospy.Publisher('/set_pose', PoseWithCovarianceStamped, queue_size=1)
	rospy.init_node('rosbot_1', anonymous=True)
	rospy.Subscriber("/odom", Odometry, odomCallback)
	rospy.Subscriber("/scan", LaserScan, scanCallback)
	rate = rospy.Rate(10)
	i=0

	# Reset the odom
	while(i<20):
		i=i+1
		odom_pub.publish(odom_reset)
		rate.sleep()

	while not rospy.is_shutdown():

		if ((abs(x-xg) < 0.05) and (abs(y-yg) < 0.05)) or obstacle:
			vel.linear.x = 0
			vel.angular.z = 0
			vel_pub.publish(vel)
			break

		vd = vx + pid(0.3,vx)
		
		theta = atan2(yg-y,xg-x)

		vel.linear.x = min(max(vd,-v_max), v_max)
		vel.angular.z = atan2(sin(theta - yaw), cos(theta-yaw))
		vel_pub.publish(vel)

		#rospy.loginfo(" x: %f , y: %f , Vel: %f , Ang: %f", x, y, vel.linear.x,vel.angular.z)

		rate.sleep()
		

if __name__ == "__main__":
	main()
