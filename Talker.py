#!/usr/bin/env python
import rospy
import random
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from p2os_msgs.msg import SonarArray
from p2os_msgs.msg import MotorState

import sys
import time
import math

"""
MRP HW3b Sameer Pawar (SLP3706)
Runtime Input argument: <String> FileName containing locations
"""



class Node:
    """
    Class for handling instance variables and to avoid usage of global variables
    """
    __slots__ = "sonar_data", "laser_data", "pub", "goal", "locations", "done", "flag", "flag2", "safe", "angle", "pub_start", "direct", "wall_dist"

    # Publisher for cmd_vel (Moves the robot)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    # Publisher for motor state (On when '1' is published)
    pub_start = rospy.Publisher('/cmd_motor_state', MotorState, queue_size=1)
    # Flag for whether all locations are covered
    done = False
    direct = 0
    locations = []

    def __init__(self, argv1):
        """
        argv1 is the command line input for file name
        """
        print("PUBLISHER STARTED")
        # Read input file
        file = open(argv1, 'r')
        for line in file:
            coord = line.split()
            if len(coord) > 1:
                coord = [float(i) for i in coord]
                self.locations.append(coord)

        # Initialize first goal
        self.goal = self.locations.pop(0)
        # Set flags to let laser scan
        # self.flag = 1
        # self.flag2 = 1
        self.safe = True
        self.angle = 0
        self.sonar_data = None

        self.wall_dist = -1

        self.pub_start.publish(1)
        # Initialize the subscribers
        rospy.Subscriber('/pose', Odometry, self.callback)
        # rospy.Subscriber('/kinect_laser/scan', LaserScan, self.laserback)
        rospy.Subscriber('/sonar', SonarArray, self.sonarback)
        # Let them listen as long as the node is running
        rospy.spin()

    def sonarback(self, data):
        """
        Callback function for our Subscriber to Sonar data
        :param data: Array of distances
        :return: None
        """
        # print("sonarback")
        self.sonar_data = data.ranges
        # Check if safe and find direction to go in if not safe
        direct = self.isSafe(data.ranges)
        if self.safe:
            self.direct = 0
        else:
            self.direct = direct
        # self.flag2 = 0
        # while self.flag2 is 0:
            # rospy.Rate(2).sleep()

    def callback(self, data):
        """
        Callback function for our Subscriber to Odometry data.
        We use this function to constantly update our robot's route ...
        every time callback is invoked
        """

        # input_message_type = str(data._type)
        # print(input_message_type)
        self.current_pos = data.pose.pose
        # print(data.pose.pose)

        self.pub_start.publish(1)
        if self.done is False:
            # Get current angle of robot using Quaternion
            current_theta = 2 * math.atan2(self.current_pos.orientation.z, self.current_pos.orientation.w)
            # Get values of the portion in which robot can fit with safe margin and average them
            # self.flag = 1
            # self.flag2 = 1
            rospy.Rate(1).sleep()

            if not self.safe:
                print("NOT safe")
                # Check whether direction is set
                if self.angle is 0:
                    if self.direct is 1:
                        # set turn-to-angle
                        self.angle = current_theta - 1.25
                    else:
                        self.angle = current_theta + 1.25
                elif not self.angle - 0.1 < current_theta < self.angle + 0.1:
                    # Keep turning till we align with turn-to-angle
                    twist = Twist()
                    twist.linear.x = 0
                    twist.linear.y = 0
                    twist.angular.z = -0.10 if self.direct is 1 else 0.10
                    self.pub.publish(twist)
                else:
                    # Follow the wall
                    if self.wall_dist is -1:
                        self.wall_dist = self.sonar_data[0] + 0.25 if self.direct is 1 else self.sonar_data[7] + 0.25
                    # Right direction turn + wall following
                    if self.direct is 1:
                        # Keep following the wall till its safe to turn
                        if self.sonar_data[0] <= self.wall_dist + 1:
                            twist = Twist()
                            twist.linear.x = 0.25
                            twist.linear.y = 0
                            twist.angular.z = 0
                            self.pub.publish(twist)
                        else:
                            self.safe = True
                    # Left direction turn + wall following
                    else:
                        if self.sonar_data[7] < self.wall_dist + 1:
                            twist = Twist()
                            twist.linear.x = 0.25
                            twist.linear.y = 0
                            twist.angular.z = 0
                            self.pub.publish(twist)
                        else:
                            self.safe = True

            else:
                print("Safe")
                # Get X,Y,Theta of robot
                self.wall_dist = -1
                x = self.current_pos.position.x
                y = self.current_pos.position.y
                z = current_theta
                goal_x = self.goal[0]
                goal_y = self.goal[1]
                # Compute straight line distance between current position and goal
                hypotenuse = math.sqrt((x - goal_x) ** 2 + (y - goal_y) ** 2)
                # Adaptive SPEED
                if hypotenuse >= 3:
                    linear_speed = 0.2
                    angular_speed = 0.2
                elif 3 > hypotenuse >= 2:
                    linear_speed = 0.15
                    angular_speed = 0.15
                elif 2 > hypotenuse > 1:
                    linear_speed = 0.1
                    angular_speed = 0.1
                elif hypotenuse <= 1:
                    linear_speed = 0.05
                    angular_speed = 0.05
                # Compute edges of the triangle formed
                side_x = goal_x - x
                side_y = goal_y - y
                # Get the angle of hypotenuse wrt X axis
                goal_z = math.atan2(side_y, side_x)
                # Get delta angle that our robot has to turn around
                dz = goal_z - z
                if abs(x - goal_x) < 0.3 and abs(y - goal_y) < 0.3:
                    dz = 0
                twist = Twist()
                if math.fabs(dz) > 0.2:
                    # Rotate if dz is greater than 0.2 radians threshold
                    twist.linear.x = 0
                    twist.linear.y = 0
                    # Adjust the angle in the range -2Pi to +2Pi to avoid making a complete
                    # circle every turn
                    if dz > 0 and math.fabs(dz) < math.pi:
                        # turn left if goal is on left side
                        twist.angular.z = angular_speed
                    elif dz > 0 and math.fabs(dz) >= math.pi:
                        # turn right
                        twist.angular.z = -1 * angular_speed
                    elif dz < 0 and math.fabs(dz) < math.pi:
                        # Turn right
                        twist.angular.z = -1 * angular_speed
                    else:
                        # Else turn left
                        twist.angular.z = angular_speed
                    self.pub.publish(twist)
                else:
                    # Angle is accurate now move in straight line
                    if abs(hypotenuse) > 0.3:
                        # Move forward if distance is greater than 0.3 threshold
                        twist.linear.x = linear_speed
                        twist.angular.z = 0
                        self.pub.publish(twist)
                    else:
                        # Else we are at our goal coordinates
                        print("Goal =(" + str(goal_x) + "," + str(goal_y) + ") & Reached =(" + str(x) + "," + str(y) + ")")
                        self.angle = 0

                        if len(self.locations) > 0:
                            self.goal = self.locations.pop(0)
                            print("Next goal: " + str(self.goal))
                        else:
                            # Stop the robot when no goal is set
                            twist.linear.x = 0
                            twist.linear.y = 0
                            twist.angular.z = 0
                            self.pub.publish(twist)
                            self.done = True

    def isSafe(self, rangeL):
        """
        Check whether it's safe to move straight given the Sonar Data
        :param rangeL: Sonar Data
        :return: direction in which the robot should turn to avoid the obstacle
        """
        direct = 0
        thresh = 0 if self.safe else 1
        for i in range(8):
            if rangeL[i] < 0.5 + thresh:
                if i < 4:
                    direct = 1
                else:
                    direct = -1
                # print("False")
                self.safe = False
                return direct
        # print("True")
        self.safe = True
        return direct


def main(argv1):
    # Initialize out robot node and pass filename containing locations
    node = Node(argv1)


if __name__ == '__main__':
    if len(sys.argv) < 1:
        print("Please provide cmd input filename")
    else:
        # Initialize the node
        rospy.init_node('Talker', anonymous=True)
        main(sys.argv[1])
