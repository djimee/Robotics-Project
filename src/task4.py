#! /usr/bin/python

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import actionlib

# Import all the necessary ROS message types:
from com2009_actions.msg import SearchFeedback, SearchResult, SearchAction
from sensor_msgs.msg import LaserScan

# Import some other modules from within this package
from move_tb3 import MoveTB3
from tb3_odometry import TB3Odometry

# Import some other useful Python Modules
from math import pi, radians
import numpy as np


class Task4(object):
    feedback = SearchFeedback()
    result = SearchResult()

    def __init__(self):
        rospy.init_node('task4')

        self.scan_subscriber = rospy.Subscriber(
            "/scan", LaserScan, self.scan_callback)

        self.robot_controller = MoveTB3()
        self.robot_odom = TB3Odometry()

        # initialise initial distance values
        self.distance_front = 1.0
        self.distance_right = 1.0
        self.distance_left = 1.0
        self.object_distance = 1.0
        self.front_left = 1.0
        self.front_right = 1.0

        # variable to count three-way turns
        self.threeway_turn_num = 0

        self.r = rospy.Rate(1000)
        self.ctrl_c = False

    def scan_callback(self, scan_data):
        # get scan data for straight ahead, left and right
        self.distance_left = scan_data.ranges[90]
        self.distance_right = scan_data.ranges[-90]
        self.distance_front = scan_data.ranges[0]

    # method to change linear/angular velocities and publish to controller
    def change_vels(self, linear, angular):
        self.robot_controller.set_move_cmd(linear, angular)
        self.robot_controller.publish()

    def turn(self, direction):
        self.robot_controller.stop()
        # rotate robot by 90deg in given direction
        if direction == "left":
            self.change_vels(0, 1.0)
        elif direction == "right":
            self.change_vels(0, -1.0)

        # wait for the robot to do a 90deg turn
        rospy.sleep(pi/2)
        self.robot_controller.stop()

    # method to move the robot forward and re-adjust
    def recentre(self, d_left, d_right):
        print("re-centering robot")
        
        # set robots target angle 
        curr_angle = self.robot_odom.yaw
        target_angle = 0
        if abs(curr_angle) < 45:
            target_angle = 0
        elif curr_angle > 45 and curr_angle < 135:
            target_angle = 90
        elif abs(curr_angle) > 135:
            target_angle = 180
        elif curr_angle < -45 and curr_angle > -135:
            target_angle = -90

        angular_vel = radians(target_angle - curr_angle)

        # re-centre robot
        if d_right < 0.25:
            angular_vel += 0.4
        elif d_left < 0.25:
            angular_vel -= 0.4

        # adjust angular velocity to re-centre
        self.change_vels(0.25, angular_vel)

    # system main loop
    def main_loop(self):
        # turning truth table
        # (https://www.allaboutcircuits.com/projects/how-to-build-a-robot-follow-walls/
        # notF = move forwards
        # F, R = turn left
        # F, L = turn right
        while not self.ctrl_c:
            # set direction variables
            d_front = self.distance_front
            d_right = self.distance_right
            d_left = self.distance_left

            # if front wall not too close, move forward
            if d_front > 0.5:
                print("robot moving forward")
                self.recentre(d_left, d_right)

            # if front and left walls too close, turn right
            elif d_front < 0.6 and d_left < 0.55:
                print("robot turning right")
                self.turn("right")

            # if front and right walls too close turn left
            elif (d_front < 0.6 and d_right < 0.55):
                print("robot turning left")
                self.turn("left")

            # check for a three-way turn and turn accordingly
            elif (d_front < 0.6 and d_right > 0.55 and d_left > 0.55):
                print("at a three-way turn")
                if self.threeway_turn_num == 0:
                    print("robot turning left")
                    self.turn("left")
                    self.threeway_turn_num += 1
                elif self.threeway_turn_num == 1:
                    print("robot turning right")
                    self.turn("right")
                    self.threeway_turn_num += 1
                elif self.threeway_turn_num == 2:
                    print("robot turning left")
                    self.turn("left")
                else:
                    continue
            else:
                continue

    # shutdownhook to allow ctrl+c to stop the program
    def shutdownhook(self):
        self.shutdown_function()
        self.ctrl_c = True


if __name__ == '__main__':
    task4 = Task4()
    try:
        task4.main_loop()
    except rospy.ROSInterruptException:
        pass
