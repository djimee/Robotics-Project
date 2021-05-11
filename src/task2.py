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
import math
import numpy as np

class colour_search(object):

    def __init__(self):
        rospy.init_node('turn_and_face')
        self.base_image_path = '/home/student/myrosdata/week6_images'
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()

        self.robot_controller = MoveTB3()
        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.1
        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)
        self.move_rate = '' # fast, slow or stop
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)
        self.rate = rospy.Rate(5) 
        self.m00 = 0
        self.m00_min = 100000
        
        self.color_name = ""
        self.lower_bound = []
        self.upper_bound = []

        self.mask = 
        self.hsv_img = 
        
        self.ready = False
        self.face_correct = False
        
    
    def get_init_colour(self):
        colour_data = {
            "Blue":   ([115, 224, 100],   [130, 255, 255]),
            "Red":    ([0, 185, 100], [10, 255, 255]),
            "Yellow": ([28, 128, 100], [31, 253, 255]),
            "Green":   ([25, 150, 100], [70, 255, 255]),
            "Turquoise": ([75, 150, 100], [100, 255, 255]),
            "Purple":   ([146, 162, 100], [154, 254, 255])
        }

        for colour_name, (lower, upper) in colour_data.items():
            lower_bound = np.array(lower)
            upper_bound = np.array(upper)
            mask = cv2.inRange(self.hsv_img, lower_bound, upper_bound)
            if mask.any():
                self.colour_name = colour_name
                self.lower_bound = lower_bound
                self.upper_bound = upper_bound
                print("SEARCH INITIATED: The target colour is {}".format (self.colour_name))
                break
                
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
        self.hsv_img = hsv_img

        if len(self.lower_bound) > 0 and len(self.upper_bound) > 0:
            self.mask = cv2.inRange(hsv_img, self.lower_bound, self.upper_bound) 

        m = cv2.moments(self.mask)
            
        self.m00 = m['m00']
        self.cy = m['m10'] / (m['m00'] + 1e-5)

        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)
        
        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)
        
    def change_vels(self, linear, angular):
        rospy.sleep(2)
        self.robot_controller.set_move_cmd(linear, angular)
        self.robot_controller.publish()
        rospy.sleep(2)
        self.robot_controller.stop()
        
    def move_forward(self):
        self.robot_controller.set_move_cmd(0.2, 0)    
        self.robot_controller.publish()
        rospy.sleep(5)
        self.robot_controller.stop()

    def main(self):
        while not self.ctrl_c:
            if self.ready == False:
               self.change_vels(0.0, 1)
               self.get_init_colour()
               self.change_vels(0.0, -1)
               self.move_forward()
               self.change_vels(0.0, 1.1)
               self.ready = True
               
            else:
                if self.m00 > self.m00_min and self.face_correct == False:
                # blob detected
                    if self.cy >= 560-100 and self.cy <= 560+100:
                        if self.move_rate == 'slow':
                            self.move_rate = 'stop'
                            print("SEARCH COMPLETE: The robot is now facing the target pillar.")
                            self.face_correct = True
                    else:
                        self.move_rate = 'slow'
                elif self.face_correct == True:
                    self.robot_controller.stop()
                    break  
                else:
                    self.move_rate = 'fast'
                
                if self.face_correct == False:
                    if self.move_rate == 'fast':
                        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)
                    elif self.move_rate == 'slow':
                        self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
                    elif self.move_rate == 'stop':
                        self.robot_controller.set_move_cmd(0.0, 0.0)
                    else:
                        self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
            
                    self.robot_controller.publish()
                    self.rate.sleep()
            
if __name__ == '__main__':
    search_ob = colour_search()
    try:
        search_ob.main()
    except rospy.ROSInterruptException:
        pass