# pointcloud_postprocessing



    
## Table of contents
* [General info](#general-info)
* [Technologies](#technologies)
* [Codes](#content)

## General info
This repository is mainly about post-processing pointcloud's frame obtained from Depth camera (ex. Photoneo) and RGB-D (ex. Intel Realsense). 
It assumes when pointcloud is generated from the moving frame (ex. End Effector Frame, Tool Frame), containing inconsistent calibration errors from the camera.


## Technologies
Project is created with:
* python version: 3.8
	

## Content
### contruct_3d_model.py

This code is for constructing 360[deg] 3D object models scanned with Robot.\
**Input** : 0,1, ... , N scans pointcloud scanned from different angles.\
**What does it do?** : 1) From 0,1, ... , N scans, it stitches pointcloud iteratively. \
  2) For each iteration (for each scab), it compensates calibration error through ICP. I suggest doing ICP with Robot hand where EE frame is attached. \
    ** why do we need this step: It's impossible to make the perfect camera-robot calibration.
  The imperfection creates inconsistent errors in camera-EE transform depends on robot joint config.
  We need to fix this error through pose-processing to stitch pointclouds seamlessly.

