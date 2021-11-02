from utility.utility import *
import os

## GLOBAL PARAMETERS ##
DATA_DIR = 'data/scraping_wok/gt_1'
DATA_SAVE_DIR = 'data/scraping_wok/gt_1_tf'
TARGET_DIR = 'data/nominal_pcd/panda_hand.ply'


if __name__ == '__main__':
    cp = coordinate_postprocessing(TARGET_DIR, icp_threshold = 2.5, visualize= False)
    cur_path = os.getcwd()
    data_path = os.path.join( cur_path, DATA_DIR)
    save_path = os.path.join( cur_path, DATA_SAVE_DIR )


    _, _, filenames = next(os.walk(data_path)) ## Infer Dataset

    for deform_idx, f_n in enumerate(filenames):
        cp.segment(os.path.join(data_path,f_n))
        cp.icp()
        cp.save_result(os.path.join(save_path, f"{f_n.split('/')[0]}"))  ## Infer Dataset
