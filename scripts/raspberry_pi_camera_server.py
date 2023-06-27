#! /usr/bin/env python

## @package raspberry_pi_camera
# The ROS node for controlling the Raspberry PI camera
#
# Defines a ROS action server for taking photos with the Raspberry PI camera

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

## RaspberryPiCameraServer class
#
# Defines a ROS action server for taking photos with the Raspberry PI camera
class RaspberryPiCameraServer:
  '''
    Class for camera control
  '''

  ## Constructor of RaspberryPiCameraServer class
  # @param width int type with of the image (default value is 1920)
  # @param height int type height of the image (default value is 1080)
  def __init__(self, width=1920, height=1080):
    ## @var width
    #  Image width
    self.width = width

    ## @var height
    #  Image height
    self.height = height

    ## @var bridge
    #  CvBridge() object for conversions between numpy arrays and ROS Image message types
    self.bridge = CvBridge()

    ## @var server
    #  The ROS action server for the RaspberryPiCameraAction action. The name of the action is "raspberry_pi_camera"
    self.server = actionlib.SimpleActionServer('raspberry_pi_camera', RaspberryPiCameraAction, self.execute, False)
    self.server.start()

  ## RaspberryPiCameraAction callback
  # This function gets called whenever the ROS action server receives a goal from a client
  # @param goal bark_msgs/RaspberryPiCameraGoal type action goal, it is an empty message to signal the need for an image (see action definition for further details)
  def execute(self, goal):
    '''
    RaspberryPiCameraAction callback

    goal: bark_msgs/RaspberryPiCameraGoal, action goal, it is an empty message to signal the need for an image (see action definition for further details)
    '''
    camera = picamera.PiCamera()
    time.sleep(0.1) # camera warm up
    
    camera.resolution = (1920, 1088)  # it is necessary to give 1088 height to a right image
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