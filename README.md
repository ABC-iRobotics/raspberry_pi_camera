# ROS package for controlling Raspberry camera

This ROS package implements a ROS Action Server, which uses the `RaspberryPiCamera.action` Action Message.

The camera control ROS package is responsible for taking photos with the camera upon request. The image request is a simple empty message, but in the future, this can be expanded with certain requested parameters for an image, such as white balance levels, resolution, etc.

The Action Message goal of the `RaspberryPiCamera.action` is an empty message. The result is the follow:
- `sensor_msgs/Image image` - the output image

The **raspberry_pi_camera_server.launch** file is located in **launch** folder. It defines and launches the `raspberry_pi_camera` ROS node. This node can send an image from Raspberry camera. The **laser_and_camera.launch** launches both the camera and the laser node.

The **raspberry_pi_camera_server.py** in the **scripts** folder implements the expected functionality. With the help of the **test_client.py** you can test the server.