#! /usr/bin/env python

import rospy
import actionlib
from cv_bridge import CvBridge
from bark_msgs.msg import RaspberryPiCameraAction, RaspberryPiCameraResult

import numpy as np
import time
import picamera

class RaspberryPiCameraServer:
  def __init__(self, width=3280, height=2464):
    self.width = width
    self.height = height
    self.camera = picamera.PiCamera()
    self.camera.resolution = (width, height)
    self.camera.framerate = 24
    time.sleep(2)

    self.bridge = CvBridge()
    self.server = actionlib.SimpleActionServer('raspberry_pi_camera', RaspberryPiCameraAction, self.execute, False)
    self.server.start()

  def execute(self, goal):
    image = np.empty((self.width * self.height * 3,), dtype=np.uint8)
    self.camera.capture(image, 'bgr')
    image = image.reshape((self.height, self.width, 3))

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