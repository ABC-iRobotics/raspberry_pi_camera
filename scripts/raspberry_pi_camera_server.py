#! /usr/bin/env python

import rospy
import actionlib
from cv_bridge import CvBridge
from bark_msgs.msg import RaspberryPiCameraAction, RaspberryPiCameraResult

import numpy as np
import time
import picamera
from picamera.array import PiRGBArray
from fractions import Fraction

class RaspberryPiCameraServer:
  #def __init__(self, width3280, height=2464):
  def __init__(self, width=1920, height=1080):

    self.width = width
    self.height = height
    # self.camera = picamera.PiCamera()
    # self.camera.resolution = (width, height)
    # time.sleep(2)
    # self.rawCapture = PiRGBArray(self.camera)

    self.bridge = CvBridge()
    self.server = actionlib.SimpleActionServer('raspberry_pi_camera', RaspberryPiCameraAction, self.execute, False)
    self.server.start()



  # def __del__(self):
  #   self.camera.close()

  def execute(self, goal):
    camera = picamera.PiCamera()
    camera.resolution = (self.width, self.height)
    camera.iso = 100
    a = Fraction(313, 256)
    b = Fraction(350, 128)
    gains = [a, b]
    time.sleep(2)

    camera.exposure_mode = 'off'
    camera.awb_mode = 'off'
    camera.awb_gains = gains
    camera.shutter_speed = 15000#3000
    time.sleep(2)

    rawCapture = PiRGBArray(camera)
    rawCapture.truncate(0) # ez itt valamiért nagyon jó lesz
    camera.capture(rawCapture, 'bgr')
    rawCapture.truncate(0) # ez itt valamiért nagyon jó lesz

    image = rawCapture.array
    image = np.reshape(image, (self.height, self.width, 3))
    result = RaspberryPiCameraResult()
    result.image = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
    rawCapture.truncate(0) # ez itt valamiért nagyon jó lesz
    camera.close()
    
    self.server.set_succeeded(result)

if __name__ == '__main__':
  rospy.init_node('raspberry_pi_camera_server')
  node_name = rospy.get_name()
  width = rospy.get_param("/" + node_name + "/width")
  height = rospy.get_param("/" + node_name + "/height")

  server = RaspberryPiCameraServer(width, height)
  rospy.spin()