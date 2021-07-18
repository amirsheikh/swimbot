# import the necessary packages
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import numpy as np

from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2

rospy.init_node("imagetimer111", anonymous=True)
pub = rospy.Publisher('imagetimer', Image,queue_size=10)
br = CvBridge()

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
time.sleep(0.1)
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	image = frame.array
    pub.publish(br.cv2_to_imgmsg(image))
	cv2.imshow("Frame", image)
	key = cv2.waitKey(1) & 0xFF
	rawCapture.truncate(0)
    if key == ord("q"):
		break

