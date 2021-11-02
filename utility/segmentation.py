import open3d as o3d
import copy
import numpy as np

class segment_component():
    def __init__(self, pcd):
        self.pcd_hand = self.hand(pcd)
        self.pcd_object = self.object(pcd)
        super().__init__()


    def hand(self, pcd, bbox = None):
        bbox = [[-100, -120, -110], [100, 120, -30]] if bbox is None else bbox
        bbox = o3d.geometry.AxisAlignedBoundingBox(bbox[0], bbox[1])
        pcd = copy.deepcopy(pcd).crop(bbox)
        # o3d.visualization.draw_geometries([ pcd ] )

        # rgb = np.array(pcd.colors)
        # idx = np.where(np.average(rgb, axis=1) > 0.15)[0]
        # pcd = pcd.select_by_index(idx)

        # cl, ind = pcd.remove_radius_outlier(nb_points=70, radius=5)
        # pcd = pcd.select_by_index(ind)
        return pcd

    def object(self, pcd, bbox = None):
        bbox = [[-50, -70, 20], [200, 70, 700]] if bbox is None else bbox
        bbox = o3d.geometry.AxisAlignedBoundingBox(bbox [0], bbox [1])
        pcd = copy.deepcopy(pcd).crop(bbox)
        # rgb = np.array(pcd.colors)
        # idx = np.where(np.average(rgb, axis=1) > 0.2)[0]
        # pcd = pcd.select_by_index(idx)

        cl, ind = pcd.remove_radius_outlier(nb_points=70, radius=5)
        pcd = pcd.select_by_index(ind)

        # cl, ind = pcd.remove_radius_outlier(nb_points=150, radius=5)
        # pcd = pcd.select_by_index(ind)

        return pcd
