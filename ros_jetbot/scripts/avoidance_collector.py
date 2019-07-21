#!/usr/bin/env python3
import cv2
import os
from uuid import uuid1
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Joy, Image


class AvoidanceCollector:

    image = None
    blocked_dir = 'dataset/blocked'
    free_dir = 'dataset/free'
    _bridge = CvBridge()

    def __init__(self, parents_dir):
        rospy.Subscriber('/joy', Joy, self.joy_stick_callback)
        rospy.Subscriber('/jetbot/image_raw', Image, self.image_callback)
        rospy.init_node("avoidance_collector")
        parents_dir = rospy.get_param('~save', '/tmp/dataset')
        if parents_dir != '':
            self.blocked_dir = os.path.join(parents_dir, 'blocked')
            self.free_dir = os.path.join(parents_dir, 'free')

    def start(self):
        self._create_save_falder()
        rospy.spin()

    def joy_stick_callback(self, msg):
        if msg.buttons[0] == 1:
            self._save_free()
        if msg.buttons[1] == 1:
            self._save_block()

    def image_callback(self, msg):
        _image = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.image = bytes(cv2.imencode('.jpg', _image)[1])


    def _create_save_falder(self):
        try:
            os.makedirs(self.free_dir)
            os.makedirs(self.blocked_dir)
        except FileExistsError:
            rospy.loginfo('Directories not created becasue they already exist')

    def _save_free(self):
        self.save_snapshot(self.free_dir)
        rospy.loginfo('save free')
    def _save_block(self):
        self.save_snapshot(self.blocked_dir)
        rospy.loginfo('save block')
    def save_snapshot(self, directory):
        image_path = os.path.join(directory, str(uuid1()) + '.jpg')
        with open(image_path, 'wb') as f:
            f.write(self.image)


if __name__ == '__main__':
    avoidance = AvoidanceCollector()
    avoidance.start()