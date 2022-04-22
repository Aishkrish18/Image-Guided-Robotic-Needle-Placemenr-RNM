# Image-Guided-Robotic-Needle-Placement-RNM

This project is offered by the Medical Technology and Intelligent Systems under the course Robotics and Navigation in Medicine. In this project, we need to  develop an image guided robotic needle placement system using the Robot Operating System (ROS) framework. We should use a depth camera (Kinect Azure by Microsoft) mounted on a robot arm (Panda by Franka Emika) to record 3D images of a chest phantom.We need to find the transformation between the robot's end effector and the camera with an eye-in-hand calibration. Using this transformation we need to stitch the individual images to a combined scan while the robot drives the camera around the phantom. We should then register this scan to a high resolution model, obtained from computer tomography. Within the high resolution model a target for the needle will be given. We perform trajectory planning to find a collision free and kinematically feasible path to the target. Lastly, we should exchange the camera for a needle (mock-up) and let the robot perform the insertion.

Project done in - Python
This was a group project of 6 students
