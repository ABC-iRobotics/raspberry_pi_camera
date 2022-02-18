#! /usr/bin/env python

import rospy
import actionlib
from cv_bridge import CvBridge
from bark_msgs.msg import RaspberryPiCameraAction, RaspberryPiCameraResult

import numpy as np
import time
import picamera
from picamera.array import PiRGBArray

class RaspberryPiCameraServer:
  def __init__(self, width=1920, height=1080):
    self.width = width
    self.height = height
    self.camera = picamera.PiCamera()
    self.camera.resolution = (width, height)
    time.sleep(2)
    self.rawCapture = PiRGBArray(self.camera)

    self.bridge = CvBridge()
    self.server = actionlib.SimpleActionServer('raspberry_pi_camera', RaspberryPiCameraAction, self.execute, False)
    self.server.start()

  def __del__(self):
    self.camera.close()

  def execute(self, goal):
    self.camera.capture(self.rawCapture, 'bgr')
    image = self.rawCapture.array
    image = np.reshape(image, (self.height, self.width, 3))
    result = RaspberryPiCameraResult()
    result.image = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
    
    self.server.set_succeeded(result)

if __name__ == '__main__':
  rospy.init_node('raspberry_pi_camera_server')
  node_name = rospy.get_name()
  width = rospy.get_param("/" + node_name + "/width")
  height = rospy.get_param("/" + node_name + "/height")

  server = RaspberryPiCameraServer(width, height)
  rospy.spin()