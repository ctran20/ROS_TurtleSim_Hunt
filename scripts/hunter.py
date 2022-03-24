#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

# Modified class provided by ROSWiki Tutorial
class TurtleBot:

    def __init__(self):
        # Creates a unique node
        rospy.init_node('turtle_hunter', anonymous=True)

        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',
                                                  Twist, queue_size=10)

        # Subscriber to track hunter position
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose',
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

    # Move toward runner
    def move2goal(self):
        goal_pose = Pose()

        # Get the input from the user.
        goal_pose.x = runner.x
        goal_pose.y = runner.y

        # Stop distance
        distance_tolerance = 1

        vel_msg = Twist()

        while self.euclidean_distance(goal_pose) >= distance_tolerance:
            goal_pose.x = runner.x
            goal_pose.y = runner.y

            # Linear velocity in the x-axis.
            vel_msg.linear.x = 1.6
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Log target location
            #rospy.loginfo("%s , %s" %(runner.x, runner.y))

            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

# Call back function for runner
def targetPose(data):
    runner.x = data.x
    runner.y = data.y

if __name__ == '__main__':
    runner = Pose()

    try:
        x = TurtleBot()

        # Subscriber to track runner position
        r_pose = rospy.Subscriber('/runner/pose', Pose, targetPose)

        while not rospy.is_shutdown():
            x.move2goal()
    except rospy.ROSInterruptException:
        pass