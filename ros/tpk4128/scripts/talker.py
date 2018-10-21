#!/usr/bin/env python

import numpy as np
import rospy
from sensor_msgs.msg import Image
from tpk4128.camera import Camera

def talker():
    pub = rospy.Publisher('image_webcam', Image, queue_size=10)
    rospy.init_node('webcam_image_talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    camera = Camera()
    while not rospy.is_shutdown():
        cap = camera.capture()
        image = Image()
        image.height = 480
        image.width = 640
        image.encoding = 'bgra8'
        image.step = image.width * 4
        image.data = cap.T.tostring()
        pub.publish(image)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
