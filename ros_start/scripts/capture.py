#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class CaptureNode:

    def __init__(self):

        rospy.Subscriber('jetbot/image_rect_color', Image, self.image_capture)
        rospy.Subscriber('joy', Joy, self.joy_capture)

    def image_capture(self, msg):


        pass

    def joy_capture(self, msg):

        
        pass

    def _input_disc(self,input):

        negative = -1.0 if input < 0.0 else 1.0
        input = abs(input)
        if 0.0 < input <= 0.3:
            return negative * 0.3
        elif 0.3 < input <= 0.5:
            return negative * 0.5
        elif 0.5 < input <= 0.7:
            return negative * 0.7
        elif 0.7 < input <= 1.0:
            return negative * 1.0
        else:
            return 0.0



if __name__ == '__main__':
    pass
