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
from math import sqrt, pow
import numpy as np

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image

# Import some other modules from within this package
from move_tb3 import MoveTB3

import time


class Task1ActionServer(object):
    feedback = SearchFeedback()
    result = SearchResult()

    def __init__(self):
        self.actionserver = actionlib.SimpleActionServer("/search_action_server",
                                                         SearchAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()

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

        self.robot_controller = MoveTB3()
        self.robot_odom = TB3Odometry()
        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.1
        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)

        self.cvbridge_interface = CvBridge()

        self.move_rate = ''  # fast, slow or stop
        self.target_colour = ''  # blue, red, yellow, green, turquoise or purple
        self.searching = True

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

    # method to change linear/angular velocities and publish to controller
    def change_vels(self, linear, angular):
        self.robot_controller.set_move_cmd(linear, angular)
        self.robot_controller.publish()

    def action_server_launcher(self, goal):
        r = rospy.Rate(100)
        success = True
        if not success:
            self.actionserver.set_aborted()
            return

        if self.target_colour == "none":
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
            tick2 = 18
            for i in range(tick2):
                self.robot_controller.publish()
                self.rate.sleep()
            self.robot_controller.set_move_cmd(0.0, 0.0)

        # Get the current robot odometry to work out distance travelled:
        self.posx0 = self.robot_odom.posx
        self.posy0 = self.robot_odom.posy

        # system main loop
        while True:
            # set direction variables
            d_goal = goal.approach_distance
            d_front = self.distance_front
            d_right = self.distance_right
            d_left = self.distance_left

            if self.m00 > self.m00_min:
                # blob detected
                if self.cy >= 560-100 and self.cy <= 560+100:
                    if self.move_rate == 'slow':
                        self.move_rate = 'stop'
                else:
                    self.move_rate = 'slow'
            else:
                self.move_rate = 'fast'

            # true if there is a lot of space in front, left and to the right
            if self.move_rate == 'fast' and d_front > d_goal and d_left > d_goal and d_right > d_goal:
                # check which side has more space, and change velocities depending on result
                if d_left > d_right:
                    self.change_vels(0.4, 0.8)
                    #print("SPACE IN ALL DIRECTIONS - MOVING LEFT")
                elif d_left < d_right:
                    self.change_vels(0.4, -0.8)
                    #print("SPACE IN ALL DIRECTIONS - MOVING RIGHT")

            # true if there is a lot of space in front and to the left, but not to the right
            elif self.move_rate == 'fast' and d_front > d_goal and d_left > d_goal and d_right < d_goal:
                self.robot_controller.stop()
                self.change_vels(0.5, 0.5)
                #print("TURNING LEFT - STILL SPACE IN FRONT")

            # true if there is a lot of space in front and to the right, but not to the left
            elif self.move_rate == 'fast' and d_front > d_goal and d_left < d_goal and d_right > d_goal:
                self.robot_controller.stop()
                self.change_vels(0.5, -0.5)
                #print("TURNING RIGHT - STILL SPACE IN FRONT")

            # true if there is not a lot of space in any direction
            elif self.move_rate == 'fast' and d_front < d_goal and d_left < d_goal and d_right < d_goal:
                # check which side has more space, and change velocities depending on result
                # reverse and turn because due to there being little space in front
                self.robot_controller.stop()
                self.change_vels(-0.5, 0)

                if d_left > d_right:
                    self.change_vels(0, 2)
                    #print("NOT MUCH SPACE IN ANY DIRECTION - TURNING LEFT")
                elif d_left < d_right:
                    self.change_vels(0, -2)
                    #print("NOT MUCH SPACE IN ANY DIRECTION - TURNING RIGHT")

            # true if there is not a lot of space in front, but a lot of space to the left and right
            elif self.move_rate == 'fast' and d_front < d_goal and d_left > d_goal and d_right > d_goal:
                self.robot_controller.stop()
                if d_left > d_right:
                    self.change_vels(0.05, 3.5)
                    #print("NOT MUCH SPACE IN FRONT - TURNING LEFT")
                elif d_left < d_right:
                    self.change_vels(0.05, -3.5)
                    #print("NOT MUCH SPACE IN FRONT - TURNING RIGHT")

            # true if there is a lot of space to the left and right, but not in front
            elif self.move_rate == 'fast' and d_front > d_goal and d_left < d_goal and d_right < d_goal:
                self.robot_controller.stop()
                self.change_vels(0.2, 0)
                #print("NOT A LOT OF SPACE IN FRONT - MOVING FORWARDS SLOWLY")

            # true if there is a lot of space to the left, but not in front or the right
            elif d_front < d_goal and d_left > d_goal and d_right < d_goal:
                self.robot_controller.stop()
                self.change_vels(0.1, 0.7)
                #print("TURNING LEFT - NOT MUCH SPACE IN FRONT")

            # true if there is a lot of space to the right, but not in front or the left
            elif self.move_rate == 'fast' and d_front < d_goal and d_left < d_goal and d_right > d_goal:
                self.robot_controller.stop()
                self.change_vels(0.1, -0.8)
                #print("TURNING RIGHT - NOT MUCH SPACE IN FRONT")
            elif self.move_rate == 'slow':
                print("BEACON DETECTED: Beaconing initiated.")
                self.robot_controller.stop()
                self.change_vels(0.2, 0.0)
            else:
                continue

            # calculate total distance travelled
            self.distance = sqrt(pow(
                self.posx0 - self.robot_odom.posx, 2) + pow(self.posy0 - self.robot_odom.posy, 2))

            # populate the feedback message and publish it:
            self.feedback.current_distance_travelled = self.distance
            self.actionserver.publish_feedback(self.feedback)


    # shutdownhook to allow ctrl+c to stop the program
    def shutdownhook(self):
        self.shutdown_function()
        self.ctrl_c = True


if __name__ == '__main__':
    rospy.init_node('search_action_server')
    Task1ActionServer()
    rospy.spin()
