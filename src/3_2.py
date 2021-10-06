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
vel = Twist()

v_max = 0.2
v_inc = 0.01

point = 1

xg1,yg1 = 1, 0
xg2,yg2 = 1, 1
xg3,yg3 = 0, 1
xg4,yg4 = 0, 0

xg, yg = xg1, yg1

odom_reset = PoseWithCovarianceStamped()

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
	odom_pub = rospy.Publisher('/set_pose', PoseWithCovarianceStamped, queue_size=1)
	rospy.init_node('rosbot_1', anonymous=True)
	rospy.Subscriber("/odom", Odometry, odomCallback)
	rate = rospy.Rate(10)
	
	i=0

	while(i<20):
		i=i+1
		odom_pub.publish(odom_reset)
		rate.sleep()

	while not rospy.is_shutdown():

		if (abs(x-xg) < 0.05) and (abs(y-yg) < 0.05):
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


		#vd = 0.5 * sqrt((xg-x)**2 + (yg-y)**2)
		
		theta = atan2(yg-y,xg-x)

		#vel.linear.x = min(max(vd,-v_max), v_max)
		vel.linear.x = 0.3
		vel.angular.z = atan2(sin(theta - yaw), cos(theta-yaw))
		vel_pub.publish(vel)

		rospy.loginfo(" x: %f , y: %f , Vel: %f , Ang: %f", x, y, vel.linear.x,vel.angular.z)

		rate.sleep()
		

if __name__ == "__main__":
	main()
