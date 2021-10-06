#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import Bool
from time import sleep
from random import randint
from math import atan2,sin,cos,sqrt
from tf.transformations import euler_from_quaternion as efq

yaw,x,y,vx = 0,0,0,0
vel = Twist()

v_max = 0.2
v_inc = 0.01

point = 1

xg1,yg1 = 0, 2
xg2,yg2 = 2, 2
xg3,yg3 = 2, 0
xg4,yg4 = 0.01, 0.01

xg, yg = xg1, yg1

def odomCallback(odom):
	global yaw,x,y
	quat = odom.pose.pose.orientation
	(roll,pitch,yaw) = efq([quat.x,quat.y,quat.z,quat.w])
	x = odom.pose.pose.position.x
	y = odom.pose.pose.position.y
	#vx = odom.pose.pose.velocity.x
		

def main():

	global vel, v_max, x, y, yaw, vx, xg, yg, point
	rospy.loginfo("Enter main")

	vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	rospy.init_node('rosbot_1', anonymous=True)
	rospy.Subscriber("/odom", Odometry, odomCallback)
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():

		if (abs(x-xg) < 0.1) and (abs(y-yg) < 0.1):
			point = point + 1
			if point == 2:
				xg,yg = xg2,yg2
			elif point == 3:
				xg,yg = xg3,yg3
			elif point == 4:
				xg,yg = xg4,yg4
			elif point == 5:
				vel.linear.x = 0
				vel.angular.z = 0
				vel_pub.publish(vel)
				break


		vd = 0.5 * sqrt((xg-x)**2 + (yg-y)**2)
		
		theta = atan2(yg-y,xg-x)

		vel.linear.x = min(max(vd,-v_max), v_max)
		vel.angular.z = atan2(sin(theta - yaw), cos(theta-yaw))
		vel_pub.publish(vel)

		rospy.loginfo(" x: %f , y: %f , Vel: %f , Ang: %f", x, y, vel.linear.x,vel.angular.z)

		rate.sleep()
		

if __name__ == "__main__":
	main()
