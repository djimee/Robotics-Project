#! /usr/bin/python

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image

# Import some other modules from within this package
from move_tb3 import MoveTB3

# Import the Twist message type from the geometry_msgs package:
from geometry_msgs.msg import Twist

import time

class colour_search(object):

    def __init__(self):


        rospy.init_node('turn_and_face')

        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10) # hz

        # create a Twist message instance to populate with velocity commands:
        self.vel_cmd = Twist()

        self.base_image_path = '/home/student/myrosdata/week6_images'
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()

        self.robot_controller = MoveTB3()
        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.1
        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)

        self.move_rate = '' # fast, slow or stop
        self.threshold_colour= '' # blue, red, green or turquoise

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.rate = rospy.Rate(5)

        self.m00 = 0
        self.m00_min = 10000

        self.blue_lower_threshold = (115, 225, 100)
        self.blue_upper_threshold = (130, 255, 255)

        self.red_lower_threshold = (0, 185, 100)
        self.red_upper_threshold = (10, 255, 255)

        self.green_lower_threshold = (25, 150, 100)
        self.green_upper_threshold = (70, 255, 255)

        self.turquoise_lower_threshold = (75, 150, 100)
        self.turquoise_upper_threshold = (100, 255, 255)

    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True

    def camera_callback(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        height, width, channels = cv_img.shape
        crop_width = width - 800
        crop_height = 400
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))

        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        if self.threshold_colour == 'blue':
            mask = cv2.inRange(hsv_img, self.blue_lower_threshold, self.blue_upper_threshold)
        elif self.threshold_colour == 'red':
            mask = cv2.inRange(hsv_img, self.red_lower_threshold, self.red_upper_threshold)
        elif self.threshold_colour == 'turquoise':
            mask = cv2.inRange(hsv_img, self.green_lower_threshold, self.green_upper_threshold)
        else:
            mask = cv2.inRange(hsv_img, self.green_lower_threshold, self.green_upper_threshold)

        m = cv2.moments(mask)
        self.m00 = m['m00']
        self.cy = m['m10'] / (m['m00'] + 1e-5)

        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)

        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)

    def main(self):

        self.state = "turn_left"
        self.state_change_time = time.time()
        while not self.ctrl_c:



            #Turn left, detect wall colour, and turn right to face forward again
            if self.state == "turn_left" and (time.time() - self.state_change_time) < 3:
                self.robot_controller.set_move_cmd(0.0,0.5)

            elif (time.time() - self.state_change_time) >= 3 and self.state == "turn_left":
                self.state = "turn_right"
                self.state_change_time = time.time()
                self.robot_controller.set_move_cmd(0.0,0.0)

            elif self.state == "turn_right" and (time.time() - self.state_change_time) < 3:
                self.robot_controller.set_move_cmd(0.0,-0.5)

            elif (time.time() - self.state_change_time) >= 3 and self.state == "turn_right":
                self.state = "forward"
                self.state_change_time = time.time()
                self.robot_controller.set_move_cmd(0.0,0.0)

            #Move forward to sit on the center X
            elif self.state == "forward" and (time.time() - self.state_change_time) < 2:
                self.robot_controller.set_move_cmd(0.5,0.0)

            elif (time.time() - self.state_change_time) >= 2 and self.state == "forward":
                self.state = "pillar_turn"
                self.state_change_time = time.time()
                self.robot_controller.set_move_cmd(0.0,0.0)

            #Turn 90 degrees to the left to begin the pillar object_detection_exercise
            elif self.state == "pillar_turn" and (time.time() - self.state_change_time) < 4:
                self.robot_controller.set_move_cmd(0.0,0.5)

            elif (time.time() - self.state_change_time) >= 4 and self.state == "pillar_turn":
                self.state = "detect_pillar"
                self.state_change_time = time.time()
                self.robot_controller.set_move_cmd(0.0,0.0)

            elif self.state == "detect_pillar":

                if self.m00 > self.m00_min:
                    # blob detected
                    if self.cy >= 560-100 and self.cy <= 560+100:
                        if self.move_rate == 'slow':
                            self.move_rate = 'stop'
                    else:
                        self.move_rate = 'slow'
                else:
                    self.move_rate = 'fast'

                if self.move_rate == 'fast':
                    print("MOVING FAST: I can't see anything at the moment, scanning the area...")
                    self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)
                elif self.move_rate == 'slow':
                    print("MOVING SLOW: Centering robot in front of pillar.".format(self.m00, self.cy))
                    self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
                else:
                    print("SEARCH COMPLETE: The robot is now facing the target pillar.".format(self.cy))
                    self.robot_controller.set_move_cmd(0.0, 0.0)

            self.pub.publish(self.vel_cmd)
            self.robot_controller.publish()
            self.rate.sleep()

if __name__ == '__main__':
    search_ob = colour_search()
    try:
        search_ob.main()
    except rospy.ROSInterruptException:
        pass
