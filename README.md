### workaround

'''
source ~/catkin_build_ws/install/setup.bash --extend
'''

'''
rosparam set joy_node/dev "/dev/input/js2"
rosrun image_view image_view image:=/jetbot/image_color _image_transport:=compressed
'''