#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

# Rotate turle
def rotate_turtle(ang_vel, distance):
	t0 = rospy.Time.now().to_sec()
	currDistance = 0

	vel.linear.y, vel.linear.z = 0, 0
	vel.angular.x, vel.angular.y = 0, 0

	vel.linear.x = 0
	vel.angular.z = ang_vel

	while(currDistance < distance):
		pub.publish(vel)
		t1 = rospy.Time.now().to_sec()
		currDistance = abs(ang_vel) * (t1-t0)

# Move turle forward or backward
def move_turtle(line_vel, ang_vel, distance):
	t0 = rospy.Time.now().to_sec()
	currDistance = 0

	vel.linear.y, vel.linear.z = 0, 0
	vel.angular.x, vel.angular.y = 0, 0

	vel.linear.x = line_vel
	vel.angular.z = ang_vel

	while(currDistance < distance):
		pub.publish(vel)
		t1 = rospy.Time.now().to_sec()
		currDistance = abs(line_vel) * (t1-t0)

# Set up
rospy.init_node('tutle', anonymous=True)
pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
vel = Twist()

# Draw Initials
rotate_turtle(2,2.2)
move_turtle(1, 0, 0.5)
rotate_turtle(1,0.3)
move_turtle(2, 1, 5.2)
move_turtle(2, 0, 2)
move_turtle(2, 1, 6)
move_turtle(1, 0, 0.3)
move_turtle(-1, 0, 1)
rotate_turtle(-2,2.3)
move_turtle(2, 1, 5)
move_turtle(-2, 0, 2)
move_turtle(2, 0, 4.7)
rotate_turtle(2,1.5)
move_turtle(2, 0, 2.2)
move_turtle(-2, 0, 4)
rotate_turtle(1,1)
