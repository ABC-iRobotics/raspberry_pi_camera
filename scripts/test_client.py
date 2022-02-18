#! /usr/bin/env python

import rospy
import actionlib
from cv_bridge import CvBridge

from bark_msgs.msg import RaspberryPiCameraAction, RaspberryPiCameraGoal

if __name__ == '__main__':
    rospy.init_node('raspberry_pi_camera_client')

    client = actionlib.SimpleActionClient('raspberry_pi_camera', RaspberryPiCameraAction)
    client.wait_for_server()
    bridge = CvBridge()

    goal = RaspberryPiCameraGoal()
    # Fill in the goal here
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))
    print(client.get_result())