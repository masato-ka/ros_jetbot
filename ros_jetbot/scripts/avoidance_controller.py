#!/usr/bin/env python3
import cv2

import numpy as np
import torch
import torchvision
import torch.nn.functional as F
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import rospy
from cv_bridge import CvBridge

class AvoidanceController:

    _bridge = CvBridge()
    image = None
    model_path = ''
    model = torchvision.models.alexnet(pretrained=False)
    device = torch.device('cuda')
    mean = 255.0 * np.array([0.485, 0.456, 0.406])
    stdev = 255.0 * np.array([0.229, 0.224, 0.225])
    normalize = torchvision.transforms.Normalize(mean, stdev)


    def __init__(self):
        rospy.Subscriber('/jetbot/image_raw',Image, self._image_callback)
        self._cmd_vel_pub = rospy.Publisher('/jetbot/avoidance/cmd_vel', Twist, queue_size=1)
        rospy.init_node('avoidance_controller')
        self.model_path = rospy.get_param('~model')

    def start(self):
        rate = rospy.Rate(10)
        self._publish_loop()

    def _publish_loop(self):

        while not rospy.is_shutdown():
            if self.image is None:
                rospy.logwarn("No Image subscride.")
                continue
            coli_deci = self._decide_collision()
            twist = Twist()
            if coli_deci:
                twist.angular.z = 0.5
            else:
                twist.linear.x = 0.5
            rospy.loginfo("linear_x: {}, angular_z: {}".format(twist.linear.x, twist.angular.z))
            self._cmd_vel_pub.publish(twist)

    def _decide_collision(self):
        image = self._preprocess()
        y = self.model(image)
        y = F.softmax(y, dim=1)
        prob_blocked = float(y.flatten()[0])
        return prob_blocked > 0.5


    def initialize_inferance(self):
        self.model.classifier[6] = torch.nn.Linear(self.model.classifier[6].in_features, 2)
        self.model.load_state_dict(torch.load(self.model_path))
        self.model = self.model.to(self.device)

    def _preprocess(self):
        x = self.image
        x = cv2.cvtColor(x, cv2.COLOR_BGR2RGB)
        x = x.transpose((2, 0, 1))
        x = torch.from_numpy(x).float()
        x = self.normalize(x)
        x = x.to(self.device)
        x = x[None, ...]
        return x


    def _image_callback(self, msg):
        self.image = self._bridge.imgmsg_to_cv2(msg, 'bgr8')


if __name__ == '__main__':
    ac = AvoidanceController()
    ac.initialize_inferance()
    ac.start()
