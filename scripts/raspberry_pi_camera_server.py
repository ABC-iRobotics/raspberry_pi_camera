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