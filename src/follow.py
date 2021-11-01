#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import Bool
from time import sleep
from random import randint
from math import atan2,sin,cos,sqrt
from tf.transformations import euler_from_quaternion as efq

yaw,x,y,vx = 0,0,0,0
#vel = Twist()

v_max = 0.4
v_inc = 0.01

xg = 1.5
yg = 1.5

odom_reset = PoseWithCovarianceStamped()

def odomCallback(odom):
	global yaw,x,y,goal_set, xg,yg
	quat = odom.pose.pose.orientation
	(roll,pitch,yaw) = efq([quat.x,quat.y,quat.z,quat.w])
	x = odom.pose.pose.position.x
	y = odom.pose.pose.position.y
	#vx = odom.pose.pose.velocity.x
		

def main():

	global vel, v_max, x, y, yaw, vx, xg, yg
	rospy.loginfo("Enter main")
	rospy.init_node('rosbot_1', anonymous=True)
	vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	odom_pub = rospy.Publisher('/set_pose', PoseWithCovarianceStamped, queue_size=1)
	
	rospy.Subscriber("/odom", Odometry, odomCallback)
	rate = rospy.Rate(10)
	vel = Twist()
	i = 0

	while(i<20):
		i=i+1
		odom_pub.publish(odom_reset)
		rate.sleep()

	while not rospy.is_shutdown():




		vel.linear.x = 0.2
		vel.angular.z = atan2(sin(theta - yaw), cos(theta-yaw))
		vel_pub.publish(vel)


		rate.sleep()
		

if __name__ == "__main__":
	main()
