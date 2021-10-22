from utility import *
import os

'''
This code is for constructing 360[deg] 3D object models scanned with Robot.
Input : 0,1, ... , N scans pointcloud scanned from different angles.
Condition : Input pointclouds are in End-Effector frame.
Content : 1) From 0,1, ... , N scans, it stitches pointcloud iteratively.
    2) For each iteration (for each scab), it compensates calibration error through ICP.
    I suggest doing ICP with Robot hand where EE frame is attached.
    The reason why we need this step: It's impossible to make the perfect camera-robot calibration.
    The imperfection creates inconsistent errors in camera-EE transform depends on robot joint config.
    We need to fix this error through pose-processing to stitch pointclouds seamlessly.
'''

###############################
#### SET GLOBAL PARAMETERS ####
###############################

## TODO : change the directories
'''
TARGET_DIR: Single pointcloud directory of the target scan. I usually make the target as the one with least robot hand occlusion.
SOURCE_LIST_DIR = txt file containing list of source directories 
DATA_DIR : Folder where the scans are located
DATA_SAVE_DIR : Directory where you want to save the 3D model

'''
TARGET_DIR = 'data/scraping_plastic/plastic_spatula/scan_5.ply'
SOURCE_LIST_DIR = 'config/modeling_plastic_spatula.txt'
DATA_DIR = 'data/plastic_spatula'
DATA_SAVE_DIR = 'data/nominal_pcd/plastic_spatula.ply'




if __name__ == '__main__':
    cur_path = os.getcwd()
    data_path = os.path.join( cur_path, DATA_DIR)
    save_path = os.path.join( cur_path, DATA_SAVE_DIR )
    target_path = os.path.join( cur_path, TARGET_DIR)
    source_path =  os.path.join( cur_path, SOURCE_LIST_DIR )

    source_path_list = open(source_path, "r").read().split(',\n')  ## Train dataset
    c3m = contruct_3d_model( target_path , icp_threshold = 2, visualize = True)
    c3m.stitch_pcd(source_path_list[1:])
    c3m.write_stitched_pcd(save_path)

