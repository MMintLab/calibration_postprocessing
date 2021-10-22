from utility import *
import os

## GLOBAL PARAMETERS ##
DATA_DIR = 'data/scraping/scrape_infer_805_800_1'
DATA_SAVE_DIR = 'data/scraping/transformed_infer_805_800_1'
TASK_NOMINAL_DIR = 'data/scraping/scrape_infer/scan_0.ply'
TARGET_DIR = 'data/scraping/object_model/scan_0.ply'


if __name__ == '__main__':
    cp = coordinate_postprocessing(TARGET_DIR, TASK_NOMINAL_DIR, 10, visualize= False)
    cur_path = os.getcwd()
    data_path = os.path.join( cur_path, DATA_DIR)
    save_path = os.path.join( cur_path, DATA_SAVE_DIR )


    # filenames = open("filename.txt", "r").read().split(',\n') ## Train dataset
    _, _, filenames = next(os.walk(data_path)) ## Infer Dataset

    for deform_idx, f_n in enumerate(filenames):
        cp.segment(os.path.join(data_path,f_n))
        cp.icp()

        # cp.save_result (os.path.join(save_path,f"{f_n.split('/')[0]}_{f_n.split('/')[-1].split('_')[-1]}")) ## Train dataset
        cp.save_result(os.path.join(save_path, f"{f_n.split('/')[0]}"))  ## Infer Dataset

