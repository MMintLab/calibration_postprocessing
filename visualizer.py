import numpy as np
import open3d as o3d
import os, copy
import abc
## GLOBAL PARAMETERS ##
DATA_DIR = 'scraping/transformed_infer_805_800_1'
DATA_SAVE_DIR = 'scraping/scrape_infer_805_800_1_wotable'
TASK_NOMINAL_DIR = 'scraping/scrape_gt/scan_0.ply'
TARGET_DIR = 'scraping/object_model/scan_0.ply'
CONFIG_DIR  = 'config/visualize.json'



class ICP:
    def __init__(self, source_pcd, target_pcd, threshold, trans_init = None):
        self.source_pcd = self._filter_pcd(source_pcd)
        self.target_pcd = self._filter_pcd(target_pcd)
        self.threshold = threshold

        self.trans_init = trans_init if not trans_init is None else np.eye(4)
        reg_p2p = o3d.pipelines.registration.registration_icp(source_pcd, target_pcd, threshold,self.trans_init,
                                                              estimation_method= o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                                                              criteria= o3d.pipelines.registration.ICPConvergenceCriteria(
                                                                  max_iteration=1000))

        self.icp_transformation = reg_p2p.transformation

    def _filter_pcd(self, pcd):
        cl, ind = pcd.remove_radius_outlier(nb_points=200, radius=5)
        pcd = pcd.select_by_index(ind)
        return pcd

    def draw_registration_result(self):
        source_temp = copy.deepcopy(self.source_pcd)
        target_temp = copy.deepcopy(self.target_pcd)

        o3d.visualization.draw_geometries([source_temp, target_temp])
        source_temp.paint_uniform_color([1, 0.706, 0])
        target_temp.paint_uniform_color([0, 0.651, 0.929])
        source_temp.transform(self.icp_transformation)

        o3d.visualization.draw_geometries([source_temp, target_temp])
        self.transformed_pcd = source_temp



class coordinate_postprocessing:
    def __init__(self, object_model_path, task_nominal_path, icp_threshold = 100, visualize= None):
        ## Standard Coordinate
        target_pcd = o3d.io.read_point_cloud(object_model_path)
        target_pcd = target_pcd.voxel_down_sample(voxel_size=0.5)


        self.target_palm = self.palm(target_pcd)
        self.target_object =  self.object(target_pcd)
        self.threshold = icp_threshold

        ####### REMOVE ######
        task_nominal_pcd = o3d.io.read_point_cloud(task_nominal_path)
        task_nominal_pcd = task_nominal_pcd .voxel_down_sample(voxel_size=0.5)
        self.task_nominal_palm = self.palm(task_nominal_pcd)
        self.task_nominal_object = self.object(task_nominal_pcd)

        #####################

        self.visualize = visualize
        self.off_trans = self.calculate_offset_trans()
        self.pose = None
        self.pcd = None
        self.palm_pcd = None
        self.object_pcd = None

    def calculate_offset_trans(self):
        trans_init = np.array([[-1,0,0,0],[0,-1,0,0],[0,0,1,0],[0,0,0,1]]) ## TODO
        tno_temp = copy.deepcopy(self.task_nominal_object)
        tno_temp.transform(trans_init)

        # _icp = ICP(tno_temp, self.target_object , self.threshold)
        _icp = ICP(self.task_nominal_object, self.target_object, self.threshold, trans_init )

        if self.visualize:
            _icp.draw_registration_result()
        return _icp.icp_transformation



    def segment(self, pc_path):
        '''
        Input: pointcloud directory
        :return: segmented pointcloud
        '''

        print(pc_path)
        pcd = o3d.io.read_point_cloud(pc_path)
        pcd = pcd.voxel_down_sample(voxel_size=0.001)

        self.pcd = pcd
        self.pcd = self.remove_table(copy.deepcopy(pcd))

        # self.palm_pcd = self.palm(copy.deepcopy(pcd))
        # self.object_pcd = self.object(copy.deepcopy(pcd))
        # self.interest_pcd = self.interest_zone(pcd)


        # object_pcd = crop_boundingbox(copy.deepcopy(pcd), lower_bound,upper_bound )

    def interest_zone(self,pcd):
        bbox = o3d.geometry.AxisAlignedBoundingBox([-100, -100, -280], [60, 100, 300])
        pcd = copy.deepcopy(pcd).crop(bbox)
        return pcd

    def screen_shot(self, config_dir, filename, pcd):

        # o3d.visualization.draw_geometries([pcd])
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        vis.add_geometry(pcd)

        ctr = vis.get_view_control()
        ctr.change_field_of_view(85)
        ctr.set_lookat([-19.998870849609375, 0.0007171630859375, 10.000289916992188])
        ctr.set_up([0.73593729956566911, -0.028192098426312852, 0.67646248727798042])
        ctr.set_zoom(0.6600)
        ctr.set_front([0.67695774172700673, 0.014173560755697201, -0.73588540282531489])

        vis.run()

        vis.capture_screen_image(filename)
        vis.destroy_window()

    def remove_table(self, pcd):

        bbox = o3d.geometry.AxisAlignedBoundingBox([-0.10, -100, -100], [100, 100, 100])
        pcd = copy.deepcopy(pcd).crop(bbox)

        rgb = np.array(pcd.colors)
        pcd_array = np.array(pcd.points)
        L = pcd_array.shape[0]

        idx = []
        for i in range(L):
            if pcd_array[i,2] < -0.050:
                if np.average(rgb[i,:] ) < 0.2:
                    continue
            idx.append(i)
        pcd = pcd.select_by_index(idx)

        cl, ind = pcd.remove_radius_outlier(nb_points=30, radius=0.005)
        pcd = pcd.select_by_index(ind)
        return pcd




    def palm(self, pcd):
        bbox = o3d.geometry.AxisAlignedBoundingBox([-50, -100, 30], [50, 100, 110])
        pcd = copy.deepcopy(pcd).crop(bbox)

        pcd = copy.deepcopy(pcd).crop(bbox)
        rgb = np.array(pcd.colors)
        idx = np.where( np.average(rgb, axis=1) > 0.15)[0]
        pcd = pcd.select_by_index(idx)
        return pcd



    def object(self, pcd):
        bbox = o3d.geometry.AxisAlignedBoundingBox([-50, -50, -300], [50, 50, -20])
        pcd = copy.deepcopy(pcd).crop(bbox)
        rgb = np.array(pcd.colors)
        idx = np.where( np.average(rgb, axis=1) > 0.15)[0]
        pcd = pcd.select_by_index(idx)
        return pcd


    def icp(self):
        _icp = ICP(self.palm_pcd, self.task_nominal_palm , 200)
        self.pose = _icp.icp_transformation

        if self.visualize:
            _icp.draw_registration_result()


    def save_result(self, filename):
        # self.pcd.transform(self.pose)
        # self.pcd.transform(self.off_trans)
        # self.pcd.scale(0.001, np.array([0.,0.,0.]))


        o3d.io.write_point_cloud(filename, self.pcd )





if __name__ == '__main__':
    cp = coordinate_postprocessing(TARGET_DIR, TASK_NOMINAL_DIR, 10, visualize= False)
    cur_path = os.getcwd()
    data_path = os.path.join( cur_path, DATA_DIR)
    save_path = os.path.join( cur_path, DATA_SAVE_DIR )
    config_path = os.path.join( cur_path, CONFIG_DIR )

    _, _, filenames = next(os.walk(data_path))

    for deform_idx, f_n in enumerate(filenames):
        cp.segment(os.path.join(data_path,f_n))

        file_name = f_n.split(".")[0]

        # cp.screen_shot(config_path, os.path.join(save_path,f"overlap_{file_name}.png"), cp.interest_pcd)

        # cp.icp()
        cp.save_result (os.path.join(save_path,f_n))