#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Range
from std_msgs.msg import Bool
from time import sleep
from random import randint


obst = False
turning = False
left = 100
right = 100
count = 0
vel = Twist()
nstep = 20
di = 1

def rangeCallbackLeft(dist):
	global left
	left = dist.range
	#rospy.loginfo(dist.range)

def rangeCallbackRight(dist):
	global right
	right = dist.range
	#rospy.loginfo(dist.range)

def turn():
	global count
	global vel
	global turning, left, right
	global nstep, di
	if(count == 0):
		rospy.loginfo("Obstacle!")
		count = count + 1
		turning = True
		vel.linear.x = 0
		vel.angular.z = 0
		#vel_pub.publish(vel)
		nstep = randint(15,25)
		#di = randint(0,1) * 2 - 1
		if(left < right):
			di = -1
		else:
			di = 1
	elif(count<nstep):
		rospy.loginfo("Turning")
		rospy.loginfo(count)
		count = count + 1
		turning = True
		vel.linear.x = 0
		vel.angular.z = di
	else:
		count = 0
		turning = False
		rospy.loginfo("Turn Complete")
		

def main():

	global obst
	global vel

	vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	obs_pub = rospy.Publisher('/obstacle', Bool, queue_size=1)
	rospy.init_node('rosbot_turn', anonymous=True)
	rospy.Subscriber("/range/fl", Range, rangeCallbackLeft)
	rospy.Subscriber("/range/fr", Range, rangeCallbackRight)
	rate = rospy.Rate(10)
	

	while not rospy.is_shutdown():
		rospy.loginfo(turning)
		if((left < 0.3) or (right < 0.3) or turning):
			if(count == 0):
				obs_pub.publish(True)
			rospy.loginfo("turn")
			turn()
		else:
			rospy.loginfo("Go")
			vel.linear.x = 0.3
			vel.angular.z = 0

		vel_pub.publish(vel)
		rate.sleep()
		

if __name__ == "__main__":
	main()
