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
from math import sqrt, pow, pi, radians
import numpy as np

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image

# Import some other modules from within this package
from move_tb3 import MoveTB3

import time


class Task5(object):
    feedback = SearchFeedback()
    result = SearchResult()

    def __init__(self):
        rospy.init_node('task3')

        self.scan_subscriber = rospy.Subscriber(
            "/scan", LaserScan, self.scan_callback)

        # initialise arc angles for each direction
        self.front_arc_angles = np.arange(-20, 21)
        self.right_arc_angles = np.arange(-40, -10)
        self.left_arc_angles = np.arange(25, 70)

        # initialise initial distance values
        self.distance_front = 1.0
        self.distance_right = 1.0
        self.distance_left = 1.0
        self.object_distance = 1.0
        self.direct_front = 1.0
        self.direct_left = 1.0
        self.direct_right = 1.0

        self.small_distance_front = 1.0

        self.robot_controller = MoveTB3()
        self.robot_odom = TB3Odometry()
        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.1
        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)

        self.cvbridge_interface = CvBridge()

        self.move_rate = ''  # fast, slow or stop
        self.target_colour = ''  # blue, red, yellow, green, turquoise or purple
        self.exploring = False
        self.navigating_maze = False
        self.targetting = False

        self.rate = rospy.Rate(5)

        self.m00 = 0
        self.m00_min = 10000

        self.lower = [(115, 224, 100), (0, 185, 100), (28, 128, 100),
                      (25, 150, 100), (75, 150, 100), (145, 162, 100)]
        self.upper = [(130, 255, 255), (10, 255, 255), (31, 253, 255),
                      (70, 255, 255), (100, 255, 255), (154, 255, 255)]
        self.target_colour = 'none'
        self.expected_colour = ["blue", "red",
                                "yellow", "green", "turquoise", "purple"]

        self.base_image_path = '/home/student/myrosdata/week6_images'
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
                                                  Image, self.camera_callback)

    def camera_callback(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(
                img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        height, width, channels = cv_img.shape
        crop_width = width - 800
        crop_height = 100
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))

        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        self.hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        if self.target_colour != "none":
            i = self.expected_colour.index(self.target_colour)
            mask = cv2.inRange(self.hsv_img, self.lower[i], self.upper[i])
            m = cv2.moments(mask)
            self.m00 = m['m00']
            self.cy = m['m10'] / (m['m00'] + 1e-5)
            if self.m00 > self.m00_min:
                cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)

        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)

    def scan_callback(self, scan_data):
        left_arc = scan_data.ranges[0:90]
        right_arc = scan_data.ranges[-90:]
        front_arc = np.array(right_arc[::-1] + left_arc[::-1])

        self.object_distance = front_arc.min()

        # from the front of the robot, obtain a 20 degree arc of scan data either side of the x-axis
        front_left_arc = scan_data.ranges[0:21]
        front_right_arc = scan_data.ranges[-20:]

        # combine the "left_arc" and "right_arc" data arrays, flip them so that the data is
        # arranged from left (-20 degrees) to right (+20 degrees) then convert to a numpy array
        front_arc = np.array(front_left_arc[::-1] + front_right_arc[::-1])

        # find the miniumum object distance within the frontal laserscan arc:
        self.distance_front = front_arc.min()
        self.front_angle = self.front_arc_angles[np.argmin(front_arc)]

        # from the left of the robot, obtain a 20 degree arc of scan data either side of the x-axis
        left_arc = scan_data.ranges[25:70]
        right_arc = scan_data.ranges[-40:-10]
        left_arc_arr = np.array(left_arc[::1])
        right_arc_arr = np.array(right_arc[::1])

        # find the miniumum object distance within the left and right laserscan arc:
        self.distance_left = left_arc_arr.min()
        self.distance_right = right_arc_arr.min()

        # get scan data for straight ahead, left and right
        self.direct_left = scan_data.ranges[90]
        self.direct_right = scan_data.ranges[-90]
        self.direct_front = scan_data.ranges[0]

        # get narrower scan data, used in maze navigation
        narrow_left = scan_data.ranges[0:16]
        narrow_right = scan_data.ranges[-15:]
        narrow_front = np.array(narrow_left + narrow_right)
        self.narrow_distance_front = narrow_front.min()

        small_left = scan_data.ranges[16:31]
        self.small_distance_left = np.array(small_left).min()

        small_right = scan_data.ranges[290:342]
        self.small_distance_right = np.array(small_right).min()

        left_arc = scan_data.ranges[0:10]
        right_arc = scan_data.ranges[-10:]
        front_arc2 = np.array(left_arc + right_arc)
        self.thin_front = front_arc2.min()

        # get initial robot odometry
        self.posx0 = self.robot_odom.posx
        self.posy0 = self.robot_odom.posy

        # check if the robot has moved far enough from the start point to detect the beacon
        self.in_range = False

    # method to change linear/angular velocities and publish to controller
    def change_vels(self, linear, angular):
        self.robot_controller.set_move_cmd(linear, angular)
        self.robot_controller.publish()

    # method to turn the robot given the direction
    def turn(self, direction):
        self.robot_controller.stop()
        # rotate robot by 90deg in given direction
        if direction == "left":
            self.change_vels(0, 1.0)
        elif direction == "right":
            self.change_vels(0, -1.0)
        elif direction == "backwards":
            self.change_vels(0, 1)
            rospy.sleep(pi/2)

        # wait for the robot to do a 90deg turn
        # minus 0.055 seconds because ROS overturns
        rospy.sleep((pi/2)-0.055)
        self.robot_controller.stop()

    # method to find the target colour - used at the start
    def get_target_colour(self):
        self.robot_controller.set_move_cmd(0.0, 0.6)
        tick1 = 18
        for i in range(tick1):
            self.robot_controller.publish()
            self.rate.sleep()
        self.robot_controller.set_move_cmd(0.0, 0.0)

        for i in range(6):
            mask = cv2.inRange(
                self.hsv_img, self.lower[i], self.upper[i])
            m = cv2.moments(mask)
            self.m00 = m['m00']
            if self.m00 > self.m00_min:
                self.target_colour = self.expected_colour[i]
                print("SEARCH INITIATED: The target colour is {}.".format(
                    self.target_colour))

        self.robot_controller.set_move_cmd(0.0, -0.6)
        tick2 = 17
        for i in range(tick2):
            self.robot_controller.publish()
            self.rate.sleep()
        self.robot_controller.set_move_cmd(0.0, 0.0)

    # get the displacement of the robot to ensure it doesn't stop at the back of the start zone
    def get_displacement(self):
        self.displacement = sqrt(pow(
            self.posx0 - self.robot_odom.posx, 2) + pow(self.posy0 - self.robot_odom.posy, 2))

        # return robot displacement
        return self.displacement
        
    # method for robot to navigate through the maze
    def navigate_maze(self, d_front, d_left, d_right):
        # true if robot is at a dead end and can only turn around
        if d_front < 0.2 and d_left < 0.2 and d_right < 0.2:
            print("turning backwards")
            self.turn("backwards")
        # true if there is space in front and to the right
        elif d_front > 0.325 and d_right > 0.325:
            self.change_vels(0.185, -0.81)
        # true if there is space in front but not to the right
        elif d_front > 0.325 and d_right < 0.325:
            self.change_vels(0.12, 0.5)
        # true if there is not space in front but is to the right
        elif d_front < 0.325 and d_right > 0.325:
            self.change_vels(0.0, -0.51)
        # true if there is not space in front or to the right
        elif d_front < 0.325 and d_right < 0.325:
            self.change_vels(0.0, 0.52)
        else:
            self.change_vels(0, -0.5)

    # method for the robot to explore the arena - used after robot passes the maze
    def explore(self, d_goal, d_front, d_left, d_right):
        # true if there is a lot of space in front, left and to the right
        if self.move_rate == 'fast' and d_front > d_goal and d_left > d_goal and d_right > d_goal:
            # check which side has more space, and change velocities depending on result
            if d_left > d_right:
                self.change_vels(0.4, 0.8)
            elif d_left < d_right:
                self.change_vels(0.4, -0.8)

        # true if there is a lot of space in front and to the left, but not to the right
        elif self.move_rate == 'fast' and d_front > d_goal and d_left > d_goal and d_right < d_goal:
            self.robot_controller.stop()
            self.change_vels(0.5, 0.5)

        # true if there is a lot of space in front and to the right, but not to the left
        elif self.move_rate == 'fast' and d_front > d_goal and d_left < d_goal and d_right > d_goal:
            self.robot_controller.stop()
            self.change_vels(0.5, -0.5)

        # true if there is not a lot of space in any direction
        elif self.move_rate == 'fast' and d_front < d_goal and d_left < d_goal and d_right < d_goal:
            # check which side has more space, and change velocities depending on result
            # reverse and turn because due to there being little space in front
            self.robot_controller.stop()
            self.change_vels(-0.5, 0)

            if d_left > d_right:
                self.change_vels(0, 2)
            elif d_left < d_right:
                self.change_vels(0, -2)

        # true if there is not a lot of space in front, but a lot of space to the left and right
        elif self.move_rate == 'fast' and d_front < d_goal and d_left > d_goal and d_right > d_goal:
            self.robot_controller.stop()
            if d_left > d_right:
                self.change_vels(0.05, 3.5)
            elif d_left < d_right:
                self.change_vels(0.05, -3.5)

        # true if there is a lot of space to the left and right, but not in front
        elif self.move_rate == 'fast' and d_front > d_goal and d_left < d_goal and d_right < d_goal:
            self.robot_controller.stop()
            self.change_vels(0.2, 0)

        # true if there is a lot of space to the left, but not in front or the right
        elif self.move_rate == 'fast' and d_front < d_goal and d_left > d_goal and d_right < d_goal:
            self.robot_controller.stop()
            self.change_vels(0.1, 0.8)

        # true if there is a lot of space to the right, but not in front or the left
        elif self.move_rate == 'fast' and d_front < d_goal and d_left < d_goal and d_right > d_goal:
            self.robot_controller.stop()
            self.change_vels(0.1, -0.8)

        elif self.move_rate == 'slow' and self.get_displacement > 1.5:
            print("BEACON DETECTED: Beaconing initiated.")
            self.robot_controller.stop()
            self.change_vels(0.3, 0.0)
            self.targetting = True

        elif self.targetting and (d_front < 0.5 or d_left < 0.2 or d_right < 0.2) and self.get_displacement > 1.5:
            print("BEACONING COMPLETE: The robot has now stopped.")
            self.robot_controller.stop()
            self.change_vels(0.0, 0.0)
            self.exploring = False
        else:
            pass

    # system main loop
    def main_loop(self):
        r = rospy.Rate(100)

        # get the target colour from the start zone if not found
        if self.target_colour == "none":
            self.get_target_colour()
            # move the robot out of the start zone and start navigating
            print("moving out of the start zone and re-aligning")
            while self.direct_left < 0.5 and self.direct_right < 0.5:
                self.change_vels(0.1, 0)
                rospy.sleep(0.5)
            print("robot has left the start zone")
            self.robot_controller.stop()
            self.navigating_maze = True
            print("starting maze navigation")

        # naviagate the maze while self.navigating_maze is true
        while self.navigating_maze:
            self.navigate_maze(self.narrow_distance_front,
                               self.small_distance_left, self.small_distance_right)
            for i in range(6):
                mask = cv2.inRange(
                    self.hsv_img, self.lower[i], self.upper[i])
                m = cv2.moments(mask)
                self.m00 = m['m00']
                if self.m00 > self.m00_min and self.expected_colour[i] != self.target_colour:
                    print("beacon spotted")
                    self.navigating_maze = False
                    self.exploring = True
                    print("maze navigation finshed - starting to explore and search for the beacon")

        # set the robot to exploring the arena while self.exploring is true and in range
        while self.exploring:
            if self.m00 > self.m00_min:
                # blob detected
                if self.cy >= 560-100 and self.cy <= 560+100:
                    if self.move_rate == 'slow':
                        self.move_rate = 'stop'
                else:
                    self.move_rate = 'slow'
            else:
                self.move_rate = 'fast'

            self.explore(0.4, self.distance_front,
                         self.distance_left, self.distance_right)

    # shutdownhook to allow ctrl+c to stop the program
    def shutdownhook(self):
        self.shutdown_function()
        self.ctrl_c = True


if __name__ == '__main__':
    task5 = Task5()
    try:
        task5.main_loop()
    except rospy.ROSInterruptException:
        pass
