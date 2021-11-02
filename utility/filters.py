import numpy as np
import copy

def rgb_filter_background(pcd):
    rgb = np.array(pcd.colors)
    idx = np.where(np.average(rgb, axis=1) > 0.2)[0]
    final_object_model = pcd.select_by_index(idx)
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
