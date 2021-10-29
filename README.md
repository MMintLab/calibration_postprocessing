# pointcloud_postprocessing



    
## Table of contents
* [General info](#general-info)
* [Technologies](#technologies)
* [Codes](#Codes)
	* [construct_3d_model.py](#construct_3d_model.py)  

## General info
This repository is mainly about post-processing pointcloud's frame obtained from Depth camera (ex. Photoneo) and RGB-D (ex. Intel Realsense). 
It assumes when pointcloud is generated from the *moving frame* (ex. End Effector Frame, Tool Frame), containing inconsistent calibration errors.


## Installation
```
git clone https://github.com/MMintLab/pointcloud_postprocessing.git
```

## Dependencies
Project is created with:
Installation | Version
------------ | -------------
python | 3.7.0
numpy | 1.21.2
open3d | 0.13.0

## Condes
- ### contruct_3d_model.py

This code is for constructing 360[deg] 3D object models scanned with Robot.\
**Input** : 0,1, ... , N scans pointcloud scanned from different angles.\
**Task** 
1.  From 0,1, ... , N scans, it stitches pointcloud iteratively. 
2. For each iteration (for each scab), it compensates calibration error through ICP. I suggest doing ICP with Robot hand where EE frame is attached.

