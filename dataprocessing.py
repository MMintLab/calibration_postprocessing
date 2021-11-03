from utilities.utility import *
import os

## GLOBAL PARAMETERS ##
DATA_DIR = 'data/scraping_silicone/scrape_gt_805_800'
DATA_SAVE_DIR = 'data/scraping_silicone/transformed_gt_805_800'
TARGET_DIR = 'data/nominal_pcd/silicone_panda_hand.ply'


if __name__ == '__main__':
    cp = coordinate_postprocessing(TARGET_DIR, object_seg_mode='conservative_filtering', icp_threshold = 2.5, visualize= False)
    cur_path = os.getcwd()
    data_path = os.path.join( cur_path, DATA_DIR)
    save_path = os.path.join( cur_path, DATA_SAVE_DIR )


    _, _, filenames = next(os.walk(data_path)) ## Infer Dataset

    for deform_idx, f_n in enumerate(filenames):
        f_n_i, extension = f_n.split('.')
        task = f_n_i.split('_')[-1]
        if extension == 'ply' and task != 'nominal':
            print(f_n)
            cp.segment(os.path.join(data_path,f_n))
            cp.icp()
            cp.save_result(os.path.join(save_path, f"{f_n.split('/')[0]}"))  ## Infer Dataset
