import cv2
import threading
import time

import numpy as np
import rospy
from jetbot import Robot


class MobileController:

    loop = True
    controll_thread = None
    left_v = 0.0
    right_v = 0.0
    max_radius = 20.0

    def __init__(self, wheel_distance, robot: Robot):
        self.wheel_distance = wheel_distance
        self.robot = robot

    def _controll_loop(self):

        while self.loop:
            self.robot.set_motors(self.left_v, self.right_v)
            time.sleep(0.1)

    def run(self):
        self.controll_thread = threading.Thread(target=self._controll_loop)
        self.controll_thread.start()
        pass

    def stop(self):
        self.loop = False

    def controll(self, speed, radius):

        if radius < 0.0:
            radius = -1 * self.max_radius * (1.0 + radius)
            radius = -1 * self.max_radius if radius == 0.0 else radius
            self.left_v = (-1*(radius+self.wheel_distance)*speed)/self.max_radius if radius != 0.0 else speed
            self.right_v = (-1*(radius-self.wheel_distance)*speed)/self.max_radius if radius != 0.0 else speed
        elif radius > 0.0:
            radius = self.max_radius * (1.0 - radius)
            radius = self.max_radius if radius == 0.0 else radius
            self.left_v = (radius+self.wheel_distance)*speed/self.max_radius if radius != 0.0 else speed
            self.right_v = (radius-self.wheel_distance)*speed/self.max_radius if radius != 0.0 else speed
        else:
            self.right_v = speed
            self.left_v = speed
        print("right :" + str(self.right_v) + ", left :" + str(self.left_v) )

    def sigmoid(self, x, gain=1, offset_x=0):
        return ((np.tanh(((x+offset_x)*gain)/2)+1)/2)