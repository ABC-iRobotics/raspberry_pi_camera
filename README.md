# ROS package for controlling Raspberry camera

The **raspberry_pi_camera_server.launch** file is located in **launch** folder. It defines and launches the `raspberry_pi_camera` ROS node. This node can send an image from Raspberry camera. The **laser_and_camera.launch** launches both the camera and the laser node.

The **raspberry_pi_camera_server.py** in the **scripts** folder implements the expected functionality. With the help of the **test_client.py** you can test the node.