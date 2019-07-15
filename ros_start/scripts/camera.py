#!/usr/bin/env python3

import yaml
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from jetbot import Camera
from sensor_msgs.msg import CameraInfo

#yaml_fname=''

NODE_NAME = 'jetbot_camera_node'
IMAGE_TOPIC = 'jetbot/image_raw'

class JetbotCamera:

    def __init__(self, width=640, height=480, calib_file=''):
        self.calib_file = calib_file
        self.camera = Camera(width=width,height=height)
        self.publisher = rospy.Publisher(IMAGE_TOPIC, Image, queue_size=1)
        self.bridge = CvBridge()

    def start(self):
        rospy.init_node(NODE_NAME)
        self.camera.observe(self.image_proc, names='value')
        self.camera.start()
        rospy.spin()

    def image_proc(self, change):
        image_value = change['new']
        cv2_image = self.bridge.cv2_to_imgmsg(image_value, 'bgr8')
        self.publisher.publish(cv2_image)

    def _load_camera_info(self):
        with open(self.calib_file, "r") as file_handle:
            calib_data = yaml.load(file_handle)
        # Parse
        camera_info_msg = CameraInfo()
        camera_info_msg.width = calib_data["image_width"]
        camera_info_msg.height = calib_data["image_height"]
        camera_info_msg.K = calib_data["camera_matrix"]["data"]
        camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
        camera_info_msg.R = calib_data["rectification_matrix"]["data"]
        camera_info_msg.P = calib_data["projection_matrix"]["data"]
        camera_info_msg.distortion_model = calib_data["distortion_model"]
        return camera_info_msg

if __name__ == '__main__':
    jetbot_camera = JetbotCamera()
    jetbot_camera.start()
    pass



