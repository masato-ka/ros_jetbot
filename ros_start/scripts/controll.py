#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from mobile.mobile import MobileController
from jetbot import Robot



class JetbotController:

    def __init__(self):
        robot = Robot()
        self.mobile = MobileController(10, robot)
        rospy.Subscriber('/joy', Joy, self.joy_stick_callback)
        rospy.Subscriber("jetbot/cmd_vel", Twist, self.cmd_vel_callback)

    def start(self):
        rospy.init_node('jetson_controller')
        self.mobile.run()
        rospy.spin()

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


    def cmd_vel_callback(self, msg):
        rospy.loginfo("Received a /cmd_vel message!")
        rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
        rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))
        self.mobile.controll(msg.linear.x, msg.angular.z)

    def joy_stick_callback(self, msg):
        slottle = self._input_disc(msg.axes[1])
        handle = self._input_disc(msg.axes[2])
        #buttons.5,7
        rospy.loginfo("joystick event Slottle: {}, Argument: {}".format(slottle, handle))
        self.mobile.controll(slottle, -1.0*handle)

if __name__ == '__main__':

    jetbot_controller = JetbotController()
    jetbot_controller.start()

