'''
Save Nominal Shapes from the real data
'''
from typing import Dict

from numpy import ndarray
import open3d as o3d
from sklearn.neighbors import NearestNeighbors
import torch
import numpy as np
import sys, pickle, os

sys.path.append(os.getcwd() + '/..')
from utils import  *

## Global parameter
device = 'cuda'
on_surface_samples = 5600
off_surface_samples = 10000
DATA_FOLDER = 'deform_mat'
FILE_NAME = 'data_photoneo_deform.pickle' #Save into this file








def onsurface_augment(pcd):
    ## augment on-surface points to include off-surface points as well
    on_surface_points = pcd.shape[0]

    ## Calculate Normals of on-surface points
    on_surface_normals = Normals(pcd)
    on_surface_normals = on_surface_normals / np.linalg.norm(on_surface_normals, axis=-1, keepdims=True)

    ### Off-surface points data processing
    ## 1. Sample uniform Query Points in the bounding box
    off_surface_uniform = np.random.uniform(-1, 1, size=(off_surface_samples, 3))

    # GT TSDF of the query points
    nbrs = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(pcd)
    distances, indices = nbrs.kneighbors(off_surface_uniform)

    ## 2. Near-surface query points
    near_surface_points = pcd.shape[0]
    below_idx = np.where(pcd[:, 2] < 0.2)[0]
    print(len(below_idx))
    near_surface_coords = query_near_surface(pcd)

    # Densely sample where deformation mainly happens
    near_surface_points_bz = len(below_idx) * 2
    near_surface_coords_bz1 = query_near_surface(pcd[below_idx, :], 0.04)
    near_surface_coords_bz2 = query_near_surface(pcd[below_idx, :], 0.04)

    # near_surface_points_bz = near_surface_points  * 2
    # near_surface_coords_bz1 = query_near_surface(pcd, 0.04)
    # near_surface_coords_bz2 = query_near_surface(pcd, 0.04)


    near_surface_coords_tot = np.concatenate([near_surface_coords_bz1, near_surface_coords_bz2, near_surface_coords],
                                             axis=0)
    near_surface_gt = np.ones((near_surface_points + near_surface_points_bz, 1)) * 0.04

    ## 3. Combine  1 and 2
    off_surface_coords = np.concatenate([off_surface_uniform, near_surface_coords_tot], axis=0)
    off_surface_normals = np.ones((off_surface_samples + near_surface_points + near_surface_points_bz, 3)) * -1
    off_surface_gt = np.concatenate([distances, near_surface_gt], axis=0)

    print(pcd.shape)
    coords =  np.concatenate([pcd, off_surface_coords], axis=0)
    gt = np.concatenate([ np.array([0] * on_surface_points).reshape(-1,1), off_surface_gt], axis=0)
    normals  = np.concatenate([on_surface_normals, off_surface_normals], axis=0)

    coords = torch.tensor(coords)
    gt = torch.tensor(gt)
    normals = torch.tensor(normals)

    return coords, gt, normals

def find_cnt_idx(nominal_pc, cnt_pc):
    idx_tot = []
    print(cnt_pc.shape)
    for cnt_i in cnt_pc:
        idx_array = np.argwhere(np.all((nominal_pc - cnt_i)==0, axis=1))[0]
        idx_tot.append(idx_array)
    idx_tot = np.concatenate(idx_tot, axis = 0)
    return idx_tot


def mat_add_data(dir, f_n, data, deform_idx):
    shape_idx = int(f_n.split('_')[0][-1])
    ext = f_n.split('.')[-1]
    file_dir = os.path.join(dir, f_n)

    ## Nominal Pointcloud
    obj = loadmat(file_dir)['s']
    pcd = obj['undeformed']['node']
    pcd = np.transpose(pcd)

    pcd, scale = bounding_box(pcd, zflip=False)
    scale_ = 2 / max([- scale[0, 0] + scale[0, 1], - scale[1, 0] + scale[1, 1], - scale[2, 0] + scale[2, 1]])

    on_idx = np.arange(pcd.shape[0])
    np.random.shuffle(on_idx)
    on_idx = on_idx[:5600]
    xyz_n = pcd[on_idx, :]

    ## If a new shape, save nominal info
    if shape_idx not in data.keys():
        coords_n, gt_n, normals_n = onsurface_augment(xyz_n)
        data[shape_idx] = {"nominal": {
            'coords': coords_n,
            'gt': gt_n,
            'normals': normals_n,
            'scale': scale_
        }}

    ## Deformed Pointcloud

    xyz_d_x = obj['undeformed']['node'][0, :] + obj['fea']['dsp_x']
    xyz_d_y = obj['undeformed']['node'][1, :] + obj['fea']['dsp_y']
    xyz_d_z = obj['undeformed']['node'][2, :] + obj['fea']['dsp_z']
    xyz_d_ori = np.concatenate([xyz_d_x[:, np.newaxis], xyz_d_y[:, np.newaxis], xyz_d_z[:, np.newaxis]], axis=1)
    print(xyz_d_ori.shape)
    xyz_d, _ = bounding_box(xyz_d_ori[on_idx, :], scale)

    cnt_nom = np.zeros((obj['bc']['contact_nodes']['x'].shape[0], 3))
    cnt_nom[:, 0] = obj['bc']['contact_nodes']['x']
    cnt_nom[:, 1] = obj['bc']['contact_nodes']['y']
    cnt_nom[:, 2] = obj['bc']['contact_nodes']['z']
    c_idx = find_cnt_idx(np.transpose(obj['undeformed']['node']).copy(), cnt_nom)
    cnt_xyz_d, _ = bounding_box(xyz_d_ori[c_idx, :], scale)

    coords_d, gt_d, normals_d = onsurface_augment(xyz_d)
    data[shape_idx][deform_idx] = {
        'coords': coords_d,
        'gt': gt_d,
        'normals': normals_d,
        'contact': torch.tensor(cnt_xyz_d),
        'reaction': torch.tensor(obj['fea']['reaction_force']),
        'scale': scale_
    }
    return data

def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

def ply_add_data(dir, f_n, data, deform_idx, wrench = None):

    ## TODO : update the code block with deformation
    shape_idx = int(1) ## TODO : assign special shape index to the real spatula

    file_dir = os.path.join(dir, f_n)
    o3d_pcd = o3d.io.read_point_cloud(file_dir)
    pcd = np.array(o3d_pcd.points)
    print(np.amax(pcd, axis=0), np.amin(pcd, axis=0))

    o3d_pcd = o3d_pcd.voxel_down_sample(voxel_size=0.0005)
    # o3d.visualization.draw_geometries([o3d_pcd])


    cl, ind = o3d_pcd.remove_radius_outlier(nb_points=50, radius=0.003)
    # display_inlier_outlier(o3d_pcd, ind)

    o3d_pcd = o3d_pcd.select_by_index(ind)
    pcd = np.array(o3d_pcd.points)


    # new_pcd = o3d.geometry.PointCloud()
    # new_pcd.points = o3d.utilities.Vector3dVector(xyz_n)
    # o3d.visualization.draw_geometries([new_pcd])

    ## Photoneo data name explicitly tells you the type
    if wrench is None:
        pcd, scale = bounding_box(pcd, zflip=False)
        scale_ = 2 / max([- scale[0, 0] + scale[0, 1], - scale[1, 0] + scale[1, 1], - scale[2, 0] + scale[2, 1]])

        on_idx = np.arange(pcd.shape[0])
        np.random.shuffle(on_idx)
        on_idx = on_idx[:100000]
        xyz_n = pcd[on_idx, :]

        coords_n, gt_n, normals_n = onsurface_augment(xyz_n)
        data[shape_idx] = {"nominal": {
            'coords': coords_n,
            'gt': gt_n,
            'normals': normals_n,
            'scale_': scale_,
            'scale': scale
        }}
    else:
        ## Deformed Pointcloud
        pcd, _ = bounding_box(pcd, scale = data[shape_idx]["nominal"]["scale"], zflip=False)

        on_idx = np.arange(pcd.shape[0])
        np.random.shuffle(on_idx)
        on_idx = on_idx[:100000]
        xyz_d = pcd[on_idx, :]

        coords_d, gt_d, normals_d = onsurface_augment(xyz_d)
        data[shape_idx][deform_idx] = {
            'coords': coords_d,
            'gt': gt_d,
            'normals': normals_d,
            'reaction': torch.tensor(wrench)
        }
    return data




def main(dir):
    cur_path = os.getcwd()
    os.chdir( os.path.join( cur_path, DATA_FOLDER) )
    _, _, filenames = next(os.walk(dir))

    data = {}
    for deform_idx, f_n in enumerate(filenames):
        print(f_n)
        ## (case 1) Collected from Photoneo (ply)
        filename, ext = f_n.split('.')
        if ext == 'ply':
            ply_deform_idx = filename.split('_')[-1]
            if ply_deform_idx == 'nominal':
                wrench = None
            else:
                wrench = wrench_array[int(ply_deform_idx)][0]*0.1
                print(wrench)
            data =  ply_add_data(dir, f_n, data, deform_idx, wrench)

        ## (case 2) from simulation
        elif ext == 'mat':
            data = mat_add_data(dir, f_n, data, deform_idx)

        else:
            print("Unsupported file type")

    os.chdir(cur_path)
    # print(f_n, "keys", data.keys(), len(data))
    with open(FILE_NAME, 'wb') as handle:
        pickle.dump(data, handle, protocol=pickle.HIGHEST_PROTOCOL)

if __name__ == '__main__':
    print("updated!")
    # data_path = os.path.join(os.getcwd(), 'data/nominal')
    data_path = os.path.join(os.getcwd(), DATA_FOLDER)
    main( data_path )


