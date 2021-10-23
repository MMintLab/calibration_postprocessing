from utility import *
import os

## GLOBAL PARAMETERS ##
DATA_DIR = 'data/scraping_plastic/scraping_gt_845_835'
DATA_SAVE_DIR = 'data/scraping_plastic/transformed_gt_plastic_845_835'
TARGET_DIR = 'data/nominal_pcd/plastic_spatula_pcd_w_hand.ply'


if __name__ == '__main__':
    cp = coordinate_postprocessing(TARGET_DIR, icp_threshold = 10, visualize= False)
    cur_path = os.getcwd()
    data_path = os.path.join( cur_path, DATA_DIR)
    save_path = os.path.join( cur_path, DATA_SAVE_DIR )


    _, _, filenames = next(os.walk(data_path)) ## Infer Dataset

    for deform_idx, f_n in enumerate(filenames):
        print(f_n)
        cp.segment(os.path.join(data_path,f_n))
        cp.icp()

        cp.save_result(os.path.join(save_path, f"{f_n.split('/')[0]}"))  ## Infer Dataset

