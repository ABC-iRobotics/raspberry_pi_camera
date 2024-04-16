#! /usr/bin/env python

from pickletools import uint8
from unittest import skip
import rospy
import actionlib
from cv_bridge import CvBridge
from bark_msgs.msg import RaspberryPiCameraAction, RaspberryPiCameraResult

import numpy as np
import time
import picamera
from picamera.array import PiRGBArray
from fractions import Fraction
import cv2

class RaspberryPiCameraServer:
  def __init__(self, width=1920, height=1080):
    self.width = width
    self.height = height

    self.bridge = CvBridge()
    self.server = actionlib.SimpleActionServer('raspberry_pi_camera', RaspberryPiCameraAction, self.execute, False)
    self.server.start()


  def execute(self, goal):
    camera = picamera.PiCamera()
    camera.start_preview()
    time.sleep(2) # camera warm up

    camera.resolution = (1920, 1088)  # it is necessary to give 1088 height to a right image
    # camera.iso = 100
    # time.sleep(2) # important sleep time

    # camera.exposure_mode = 'off'
    camera.awb_mode = 'auto'
    camera.awb_gains = (1.5, 3)
    # camera.shutter_speed = 20000
    # camera.iso = 200

    rospy.sleep(2)

    print(float(camera.awb_gains[0]), float(camera.awb_gains[1]))
    print(camera.exposure_speed)
    print(camera.analog_gain)
    print(camera.digital_gain)
    print(camera.exposure_mode)

    rawCapture = PiRGBArray(camera, size=(1920,1088))
    camera.capture(rawCapture, 'rgb')
    image = rawCapture.array

    image = np.reshape(image, (1088, 1920, 3))

    image = cv2.resize(image, (self.width, self.height), interpolation=cv2.INTER_LINEAR)

    result = RaspberryPiCameraResult()
    result.image = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')

    camera.close()
    self.server.set_succeeded(result)

if __name__ == '__main__':
  rospy.init_node('raspberry_pi_camera_server')
  node_name = rospy.get_name()
  width = rospy.get_param("/" + node_name + "/width")
  height = rospy.get_param("/" + node_name + "/height")

  server = RaspberryPiCameraServer(width, height)
  rospy.spin()
