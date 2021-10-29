from utility import *
import os


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
TARGET_DIR = 'data/scraping_plastic/plastic_spatula/scan_0.ply'
# SOURCE_LIST_DIR = 'config/modeling_plastic_spatula.txt'
SOURCE_LIST_DIR = 'config/modeling_plastic_spatula.txt'
DATA_DIR = 'data/plastic_spatula'
DATA_SAVE_DIR = 'data/nominal_pcd/plastic_spatula_pcd_w_hand.ply'




if __name__ == '__main__':
    cur_path = os.getcwd()
    data_path = os.path.join( cur_path, DATA_DIR)
    save_path = os.path.join( cur_path, DATA_SAVE_DIR )
    target_path = os.path.join( cur_path, TARGET_DIR)
    source_path =  os.path.join( cur_path, SOURCE_LIST_DIR )

    source_path_list = open(source_path, "r").read().split(',\n')  ## Train dataset
    c3m = contruct_3d_model( target_path , icp_threshold = 7, visualize = False) #2.5
    c3m.stitch_pcd(source_path_list[1:])
    # c3m.write_stitched_pcd(save_path, type = 'obj_w_hand') # type = 'obj_w_hand' or 'obj'

