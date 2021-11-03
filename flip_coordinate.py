'''
    Useful tool for correct some dataset having flipped axis.
'''
import numpy as np

from utilities.utility import  *
import os

## GLOBAL PARAMETERS ##
DATA_DIR = 'data/scraping_plastic/scraping_raw_845_840'
DATA_SAVE_DIR = 'data/scraping_plastic/scrape_gt_845_850'
TARGET_DIR = 'data/nominal_pcd/silicone_panda_hand.ply'
xflip = False
yflip = True
zflip = True

if __name__ == '__main__':
    root_path = os.path.abspath(os.path.join(os.getcwd(), os.pardir))
    data_path = os.path.join( root_path, DATA_DIR)


    _, _, filenames = next(os.walk(data_path)) ## Infer Dataset

    for deform_idx, f_n in enumerate(filenames):

        extension = f_n.split('.')[-1]

        if extension != 'ply':
            continue

        file_dir = os.path.join(root_path, data_path, f_n)
        pcd = o3d.io.read_point_cloud(file_dir)
        pcd_array = np.array(pcd.points)
        rgb_array = np.array(pcd.colors)

        if xflip:
            pcd_array[:,0] = - pcd_array[:,0]

        if xflip:
            pcd_array[:,1] = - pcd_array[:,1]

        if zflip:
            pcd_array[:,2] = - pcd_array[:,2]

        pcd_correct = o3d.geometry.PointCloud()
        pcd_correct.points = o3d.utility.Vector3dVector(pcd_array)
        pcd_correct.colors = o3d.utility.Vector3dVector(rgb_array)

        save_path = os.path.join(root_path, DATA_SAVE_DIR, f_n)
        o3d.io.write_point_cloud( save_path ,pcd_correct)