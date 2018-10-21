#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image

def callback(image):
    rospy.loginfo(rospy.get_caller_id() + ' I received an image of size ({},{})'.format(image.width, image.height))

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('webcam_image_listener', anonymous=True)

    rospy.Subscriber('image_webcam', Image, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
