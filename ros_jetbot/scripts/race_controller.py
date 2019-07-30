#!/usr/bin/env python3
import PIL
import torchvision.transforms as transforms
import cv2

import numpy as np
import torch
import torchvision
import torch.nn.functional as F
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import rospy
from cv_bridge import CvBridge


class Net(torch.nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        self.conv1 = torch.nn.Conv2d(3, 24, kernel_size=3)
        self.conv2 = torch.nn.Conv2d(24, 32, kernel_size=3)
        self.conv3 = torch.nn.Conv2d(32, 64, kernel_size=3)
        self.conv4 = torch.nn.Conv2d(64, 64, kernel_size=3)
        self.conv5 = torch.nn.Conv2d(64, 64, kernel_size=3)
        #self.conv2_drop = torch.nn.Dropout2d()
        self.fc1 = torch.nn.Linear(2560, 1000)
        self.fc2 = torch.nn.Linear(1000, 100)
        self.fc3 = torch.nn.Linear(100, 100)
        self.fc4 = torch.nn.Linear(100, 2)
    def forward(self, x):
        x = F.relu(self.conv1(x), 2)
        x = F.relu(F.max_pool2d(self.conv2(x), 2))
        x = F.relu(F.max_pool2d(self.conv3(x), 2))
        x = F.relu(F.max_pool2d(self.conv4(x), 2))
        x = F.relu(F.max_pool2d(self.conv5(x), 2))
        #print(x.size())
        x = x.view(-1,2560)
        x = F.relu(self.fc1(x))
        #x = F.dropout(x, training=self.training)
        x = F.relu(self.fc2(x))
        x = F.dropout(x, training=self.training)
        x = F.relu(self.fc3(x))
        x = F.dropout(x, training=self.training)
        return self.fc4(x)


class RaceController:

    _bridge = CvBridge()
    image = None
    model_path = ''
    model = torchvision.models.resnet18(pretrained=False)
    #model.fc = torch.nn.Linear(512, 2)
    device = torch.device('cuda')
    #mean = 255.0 * np.array([0.485, 0.456, 0.406])
    #stdev = 255.0 * np.array([0.229, 0.224, 0.225])
    #normalize = torchvision.transforms.Normalize(mean, stdev)
    mean = torch.Tensor([0.485, 0.456, 0.406]).cuda().half()
    std = torch.Tensor([0.229, 0.224, 0.225]).cuda().half()

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
                #rospy.logwarn("No Image subscride.")
                continue
            slottle,handle = self._decide_collision()
            twist = Twist()
            twist.linear.x = slottle
            twist.angular.z = handle
            rospy.loginfo("linear_x: {}, angular_z: {}".format(twist.linear.x, twist.angular.z))
            self._cmd_vel_pub.publish(twist)

    def _decide_collision(self):
        image = self._preprocess()
        xy = self.model(image).detach().float().cpu().numpy().flatten()
        slottle = self._input_disc(xy[0])
        handle = self._input_disc(xy[1])
        return slottle,handle


    def initialize_inferance(self):

        self.model.fc = torch.nn.Linear(512, 2)
        self.model.load_state_dict(torch.load(self.model_path))
        self.model = self.model.to(self.device)
        self.model = self.model.eval().half()

    def _preprocess(self):
        image = PIL.Image.fromarray(self.image)
        image = transforms.functional.to_tensor(image)
        image = image.numpy()[::-1].copy()
        image = torch.from_numpy(image).to(self.device).half()
        image.sub_(self.mean[:, None, None]).div_(self.std[:, None, None])
        return image[None, ...]

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

    # def _preprocess(self):
    #     x = self.image
    #     x = cv2.cvtColor(x, cv2.COLOR_BGR2RGB)
    #     x = x.transpose((2, 0, 1))
    #     x = torch.from_numpy(x).float()
    #     x = self.normalize(x)
    #     x = x.to(self.device)
    #     x = x[None, ...]
    #     return x


    def _image_callback(self, msg):
        img = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.image = cv2.resize(img, (224,224))


if __name__ == '__main__':
    rc = RaceController()
    rc.initialize_inferance()
    rc.start()
