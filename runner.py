#!/usr/bin/env python
import rospy
import random
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, SpawnRequest, Kill
from math import pow, atan2, sqrt

# Modified class provided by ROSWiki Tutorial
class TurtleBot:

    def __init__(self):
        # Creates a unique node
        rospy.init_node('turtle_runner', anonymous=True)

        self.velocity_publisher = rospy.Publisher('/runner/cmd_vel',
                                                  Twist, queue_size=10)

        # Subscriber to track runner position
        self.pose_subscriber = rospy.Subscriber('/runner/pose',
                                                Pose, self.update_pose)

        self.pose = Pose()
        self.rate = rospy.Rate(10)

    # Subscriber callback function
    def update_pose(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    # Calculate distance between hunter and runner
    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=6):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    # Moving randomly with constant velocity
    def move(self):
        """Moves the turtle to the goal."""
        goal_pose = Pose()

        # Get the input from the user.
        distance_tolerance = 1.5

        vel_msg = Twist()
        time = 0
        rotation = random.random()*2 -0.7

        while True:
            goal_pose.x = hunter.x
            goal_pose.y = hunter.y

            # Linear velocity in the x-axis.
            vel_msg.linear.x = 1.5
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = rotation

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Log target location
            # rospy.loginfo("%s , %s" %(hunter.x, hunter.y))
            # rospy.loginfo("%s" % self.euclidean_distance(goal_pose))

            # Publish at the desired rate.
            self.rate.          sleep()

            # Change rotation after 10 iteration
            if time == 10:
                time = 0
                rotation = random.random()*2 -0.6
            time += 1
            
            # Kill current turtle when got caught
            if(self.euclidean_distance(goal_pose) < distance_tolerance):
            	kill_turtle('runner')
            	break

# Spawn turtle
def spawn_turtle(x,y,theta,name):
	rospy.wait_for_service('/spawn')
	spawn_turtle = rospy.ServiceProxy('/spawn', Spawn)
	server_response = spawn_turtle(x,y,theta,name)
	return server_response.name

# Callback function for hunter
def hunterPose(data):
    hunter.x = data.x
    hunter.y = data.y

## Main
hunter = Pose()
kill_turtle = rospy.ServiceProxy('/kill', Kill)
    
try:
    # Subscriber to track hunter location
	r_pose = rospy.Subscriber('/turtle1/pose', Pose, hunterPose)

	while not rospy.is_shutdown():
		randx = random.random() * 10
		randy = random.random() * 10

        # Spawn turtle at random location
		spawn_turtle(randx, randy,0, 'runner')
		y = TurtleBot()
		y.move()
except rospy.ROSInterruptException:
    pass