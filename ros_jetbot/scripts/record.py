#!/usr/bin/env python3
import os
from uuid import uuid1

import cv2

import rospy
from sensor_msgs.msg import Image, Joy
from cv_bridge import CvBridge

class DrivRecorder():

    _bridge = CvBridge()
    image = None
    slottle = 0.0
    handle = 0.0
    record_flg = False

    def __init__(self):
        rospy.init_node('recording node')
        rospy.Subscriber('/jetbot/image_raw', Image, self._image_callback)
        rospy.Subscriber('/joy', Joy, self._joy_callback)
        hz = rospy.get_param('~rate', 10)
        self.rate = rospy.Rate(hz)
        self.save_directory = rospy.get_param('~save', '/tmp/dataset')

    def start(self):
        self._create_save_directory()
        self._recording_loop()

    def _recording_loop(self):

        while not rospy.is_shutdown():
            if self.record_flg:
                self._save_snapshot(self.save_directory, self.slottle, self.handle)
            self.rate.sleep()

    def _save_snapshot(self, directory, slottle, handle):
        file_name = str(slottle)+'-'+str(handle)+'-'+str(uuid1())+'.jpg'
        image_path = os.path.join(directory, file_name)
        if self.image == None:
            rospy.logwarn('Can not get Image message yet.')
            return
        with open(image_path, 'wb') as f:
            f.write(self.image)
            rospy.loginfo('save snapshot as %s', (image_path))

    def _create_save_directory(self):
        try:
            os.makedirs(self.save_directory)
        except FileExistsError:
            rospy.loginfo('Directories not created becasue they already exist')

    def _image_callback(self, msg):
        _image = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.image = bytes(cv2.imencode('.jpg', _image)[1])

    def _joy_callback(self, msg):

        if msg.buttons[5] == 1:
            self.record_flg = True
        else:
            self.record_flg = False

        self.slottle = self._input_disc(msg.axes[1])
        self.handle = -1.0 * self._input_disc(msg.axes[2])
        rospy.loginfo("joystick event Slottle: {}, Argument: {}".format(self.slottle, self.handle))

    def _input_disc(self, input):

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
    recorder = DrivRecorder()
    recorder.start()