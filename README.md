# ROS Jetbot package



# Install

### setup for Python3

Your are install ROS normaly. and install rospkg for Python3

```
pip3 install rospkg
```

Rebuild vision_ros for Python3



### Download & build

clone to your ros workspace src directory and catkin_make.


# Usage

'''
roslaunch ros_jetbot jetbot.launch
'''


# Topics

### subscribe Topic

| TopicName | MSG     |  Node           |
|:----------|:--------|:----------------|
| /joy      | Joy     | controll.py     |
|jetbot/cmd_vel| Twist| controll.py     |

### Publish Topic

    * jetbot/image_raw
    * jetbot/camera_info


# Camera parameta

This package incliude camera parameter as ost.yaml
Using camera is SainSmart IMX219


# License

# Contribution

# Author

@masato-ka






