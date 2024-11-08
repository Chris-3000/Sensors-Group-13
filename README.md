# Sensors Group 13
This is a project for the subject 41014 Sensors and Control for Mechatronic Systems. It involves the hand-eye calibration and visual servoing of a Dobot Magician.

## Dependencies
- MATLAB (With Camera Calibration Toolbox)
- Peter Corke's Robotics Toolbox UTS Edition
- MATLAB Support Package for USB Webcams

## Virtual Demo
Filename: virtualDemo.m<br/>
Function: Demonstrates the functionality of hand-eye calibration and visual servoing in a virtual environment.

## Pattern Detection
Filename: Camera Calibration Test/checkerboardPoseEstimation.m<br/>
Function: Demonstrates ability to identify features on an 8x5 grid in a series of images, and their respective poses.

## Pattern Tracking
Filename: checkerboardTracker.m<br/>
Function: Efficiently identifies and tracks the checkerboard features (and the checkerboard pose) in a webcam video feed.

## Hand-Eye Calibration
Filename: cameraPositionWithoutAXXB.m<br/>
Function: Takes a picture of the checkerboard mounted on the Dobot with joint state [0,0,0,0] and determines the camera's pose relative to the robot base (displaying these relative poses in a plot).

## Visual Servoing
Filename: HECSequence.m<br/>
Function: Joints of the Dobot are controlled by cartesian translations of the pattern.
