#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import Bool
from time import sleep
from random import randint
from math import atan2,sin,cos,sqrt,pi
from tf.transformations import euler_from_quaternion as efq

yaw,x,y,vx = 0,0,0,0
vel = Twist()

v_max = 0.2
v_inc = 0.01

point = 1

radius = 1
n_points = 10
goals = []
i = 0
for i in range(n_points):
	goals.append([radius*cos(2*pi*i/n_points),radius*sin(2*pi*i/n_points)])



xg, yg = radius, 0

odom_reset = PoseWithCovarianceStamped()

def odomCallback(odom):
	global yaw,x,y
	quat = odom.pose.pose.orientation
	(roll,pitch,yaw) = efq([quat.x,quat.y,quat.z,quat.w])
	x = odom.pose.pose.position.x
	y = odom.pose.pose.position.y
	#vx = odom.pose.pose.velocity.x
		

def main():

	global vel, v_max, x, y, yaw, vx, xg, yg, point, goals
	rospy.loginfo("Enter main")

	vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	odom_pub = rospy.Publisher('/set_pose', PoseWithCovarianceStamped, queue_size=1)
	rospy.init_node('rosbot_1', anonymous=True)
	rospy.Subscriber("/odom", Odometry, odomCallback)
	rate = rospy.Rate(10)
	
	i=0

	while(i<20):
		i=i+1
		odom_pub.publish(odom_reset)
		rate.sleep()
	i = 0
	while not rospy.is_shutdown():

		if (abs(x-xg) < 0.2) and (abs(y-yg) < 0.2):
			xg,yg = goals[i % n_points]
			i += 1


		#vd = 0.5 * sqrt((xg-x)**2 + (yg-y)**2)
		
		theta = atan2(yg-y,xg-x)

		#vel.linear.x = min(max(vd,-v_max), v_max)
		vel.linear.x = 0.3
		vel.angular.z = atan2(sin(theta - yaw), cos(theta-yaw))
		vel_pub.publish(vel)

		rospy.loginfo(" x: %f , y: %f , xg: %f , yg: %f , Vel: %f , Ang: %f", x, y, xg, yg, vel.linear.x,vel.angular.z)

		rate.sleep()
		

if __name__ == "__main__":
	main()
