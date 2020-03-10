#!/usr/bin/env python

import os
import rospy
import cv2
import numpy as np

from duckietown import DTROS
#duckie bot API:Motors
from duckietown_msgs.msg import WheelsCmdStamped, BoolStamped, LEDPattern
from duckietown_msgs.srv import SetCustomLEDPattern, ChangePattern
from std_msgs.msg import String

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge


class Demo3(DTROS):

    def __init__(self,node_name):
        super(Demo3,self).__init__(node_name=node_name)

        self.wheelpub = rospy.Publisher(
            '/duckiebot3/wheels_driver_node/wheels_cmd',
            WheelsCmdStamped,
            queue_size=1)

        self.camerasub = self.subscriber(
            '/duckiebot3/camera_node/image/compressed',
            CompressedImage,
            self.onImageReceived
        )

        self.chatterpub = rospy.Publisher('chatter', String,queue_size=10)

    def detect_red_line(self,hsv):
        # lower red range
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)
        # upper red range
        lower_red = np.array([170, 120, 70])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red, upper_red)
        # final mask to detect red color
        red_mask = mask1 + mask2
        image,red_contours, hierachy = cv2.findContours(red_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        if len(red_contours) == 0:
            return False
        else:
            red_threshold = 400
            red_area_list = []
            for cnt in red_contours:
                red_area_list.append(cv2.contourArea(cnt))
            red_areamax = max(red_area_list)
            if red_areamax > red_threshold:
                return True
            else:
                return False


    def detect_yellow_line(self,frame):
        hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        # lower yellow range
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        mask1 = cv2.inRange(hsv, lower_yellow, upper_yellow)
        # upper red range
        lower_yellow = np.array([22, 60, 200])
        upper_yellow = np.array([60, 255, 255])
        mask2 = cv2.inRange(hsv, lower_yellow, upper_yellow)
        # final mask to detect yellow color
        yellow_mask = mask1 + mask2

        frame = cv2.bitwise_and(frame, frame, mask=yellow_mask)
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        canny = cv2.Canny(blur, 100, 400)
        height,width = canny.shape
        mask = np.zeros_like(canny)
        polygon = np.array([[(0, height * 1 / 1.5),
                             (width, height * 1 / 2),
                             (width, height),
                             (0, height)]], np.int32)
        cv2.fillPoly(mask, polygon, 255)
        segment = cv2.bitwise_and(canny, mask)

        image, yellow_contours, hierachy= cv2.findContours(segment,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        if len(yellow_contours) == 0:
            return False
        else:
            yellow_area_list = []
            for cnt in yellow_contours:
                yellow_area_list.append(cv2.contourArea(cnt))
            yellow_areamax = max(yellow_area_list)
            yellow_areamaxid = yellow_area_list.index(yellow_areamax)

            cnt = yellow_contours[yellow_areamaxid]
            x, _, _, _ = cv2.boundingRect(cnt)
            if x >= width/2:
                return True
            else:
                return False


    def onImageReceived(self,msg):

        #Extract the color image from the message
        '''threshold the HSV image for a range of color
        https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_colorspaces/py_colorspaces.html
        '''
        np_arr = np.fromstring(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        yellow_line = self.detect_yellow_line(frame)
        red_line = self.detect_red_line(frame)

        if (yellow_line and red_line):
            self.led_react("yellow_line and red_line")
        elif(not(yellow_line) and red_line):
            self.led_react("red_line")
        elif(yellow_line and not(red_line)):
            self.led_react("yellow_line")
        else:
            self.led_react("None")

    def led_react(self,mode):
        rospy.wait_for_service('/duckiebot3/led_emitter_node/set_custom_pattern')
        try:
            rospy.sleep(1)
            service = rospy.ServiceProxy('/duckiebot3/led_emitter_node/set_custom_pattern',
                                         SetCustomLEDPattern)
            msg = LEDPattern()
            msg.color_list = ['switchedoff', 'switchedoff', 'switchedoff', 'switchedoff', 'switchedoff']
            msg.color_mask = [1, 1, 1, 1, 1]
            msg.frequency = 0
            msg.frequency_mask = [0, 0, 0, 0, 0]
            if mode == "red_line":
               msg.color_list[1] = "red"
               msg.color_list[3] = "red"
            elif mode == "yellow_line":
               msg.color_list[0] = "yellow"
               msg.color_list[2] = "yellow"
               msg.color_list[4] = "yellow"
            elif mode == "yellow_line and red_line":
               msg.color_list[1] = "red"
               msg.color_list[3] = "red"
               msg.color_list[0] = "yellow"
               msg.color_list[2] = "yellow"
               msg.color_list[4] = "yellow"
            else:
               None
            response = service(msg)
            rospy.loginfo(response)
        except rospy.ServiceException, e:
            print("Service call failed:%"%e)




if __name__ == '__main__':
    node = Demo3(node_name="demo3")
    #bridge=CvBridge()
    #image_message=bridge.cv2_to_imgmsg(cv_image,encoding="bgr8")
    #node.run()
    rospy.spin()
