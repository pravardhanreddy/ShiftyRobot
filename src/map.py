#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from time import sleep
from math import atan2,sin,cos

w,x,y,z = 0,0,0,0
p = []
q = []

def obstacleCallback(obst):
	global p,q,x,y,w,z
	#theta = atan2(2 * w * z, 1 - 2 * z * z)
	#x = x + 0.2 * cos(theta)
	#y = y + 0.2 * sin(theta)
	p.append(x)
	q.append(y)
	plt.plot(p,q,'o',color='red')
	plt.pause(0.00001)
	rospy.loginfo(p)
	rospy.loginfo(q)

def odomCallback(odom):
	global w,x,y,z
	w = odom.pose.pose.orientation.w
	z = odom.pose.pose.orientation.z
	x = odom.pose.pose.position.x
	y = odom.pose.pose.position.y

def main():

	rospy.init_node('rosbot_map', anonymous=True)
	rospy.Subscriber("/obstacle", Bool, obstacleCallback)
	rospy.Subscriber("/odom", Odometry, odomCallback)
	rate = rospy.Rate(10)
	

	while not rospy.is_shutdown():
		rate.sleep()
		

if __name__ == "__main__":
	main()
