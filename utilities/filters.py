import numpy as np
import copy

def rgb_filter_background(pcd):
    if np.array(pcd.points).shape[0] != 0:
        rgb = np.array(pcd.colors)
        idx = np.where(np.average(rgb, axis=1) > 0.25)[0]
        pcd = pcd.select_by_index(idx)
    return pcd

def rgb_mild_filter_background(pcd):
    rgb = np.array(pcd.colors)
    idx = np.where(np.average(rgb, axis=1) > 0.1)[0]
    pcd = pcd.select_by_index(idx)
    return pcd

def statistical_outlier_removal(pcd):
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=350, std_ratio=1.5)
    pcd = pcd.select_by_index(ind)

    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=350, std_ratio=1)
    pcd = pcd.select_by_index(ind)

    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=350, std_ratio=1)
    pcd = pcd.select_by_index(ind)


    return pcd


def final_noise_removal(final_object_model_):
    final_object_model = copy.deepcopy(final_object_model_)
    final_object_model = rgb_filter_background(final_object_model)

    cl, ind = final_object_model.remove_radius_outlier(nb_points=300, radius=5)
    final_object_model = final_object_model.select_by_index(ind)
    cl, ind = final_object_model.remove_radius_outlier(nb_points=200, radius=5)
    final_object_model = final_object_model.select_by_index(ind)
    cl, ind = final_object_model.remove_radius_outlier(nb_points=500, radius=10)
    final_object_model = final_object_model.select_by_index(ind)

    return final_object_model

def final_noise_removal_mild(final_object_model_):
    final_object_model = copy.deepcopy(final_object_model_)
    final_object_model = rgb_filter_background(final_object_model)

    cl, ind = final_object_model.remove_radius_outlier(nb_points=300, radius=5)
    final_object_model = final_object_model.select_by_index(ind)

    return final_object_model
