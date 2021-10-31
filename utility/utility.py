import copy
from utility.o3d_utility import *
import open3d as o3d
import numpy as np

class ICP_class:
    def ICP(self, source_pcd, target_pcd, threshold, trans_init = None):
        self.source = self._filter_pcd(source_pcd)
        self.target = self._filter_pcd(target_pcd)
        self.threshold = threshold

        self.trans_init = trans_init if not trans_init is None else np.eye(4)
        reg_p2p = o3d.pipelines.registration.registration_icp(self.source, self.target, threshold,self.trans_init,
                                                              estimation_method= o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                                                              criteria= o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=1000))



        self.icp_transformation = reg_p2p.transformation
        return self.icp_transformation

    def icp_visualizer(self):

        vis = o3d.visualization.Visualizer()
        vis.create_window()
        vis.add_geometry(self.source)
        vis.add_geometry(self.target)

        ctr = vis.get_view_control()
        ctr.change_field_of_view(60)
        ctr.set_lookat([-14.090396391511135, 5.7634012971213266, -89.068332672119141])
        ctr.set_up([0.078516645474030991, -0.46466849581268738, 0.8819967830908908])
        ctr.set_zoom(0.8)
        ctr.set_front([-0.37897176754045897, 0.80439645426745576, 0.45752239701374764])


        self.source.paint_uniform_color([1, 0.706, 0])
        self.target.paint_uniform_color([0, 0.651, 0.929])

        icp_iteration = 100
        save_image = True

        for i in range(icp_iteration):
            reg_p2p = o3d.pipelines.registration.registration_icp(self.source, self.target, self.threshold,self.trans_init,
                                                              estimation_method= o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                                                              criteria= o3d.pipelines.registration.ICPConvergenceCriteria(
                                                                  max_iteration=1))
            self.source.transform(reg_p2p.transformation)
            vis.update_geometry(self.source)


            vis.poll_events()
            vis.update_renderer()
            if save_image and not i%4:
                vis.capture_screen_image("temp_%04d.jpg" % i)
        vis.destroy_window()


    def _filter_pcd(self, pcd):
        # cl, ind = pcd.remove_radius_outlier(nb_points=100, radius=5)
        # pcd = pcd.select_by_index(ind)

        # cl, ind = pcd.remove_radius_outlier(nb_points=200, radius=5)
        # pcd = pcd.select_by_index(ind)
        return pcd

    def draw_registration_result(self):
        source_temp = copy.deepcopy(self.source)
        target_temp = copy.deepcopy(self.target)

        o3d.visualization.draw_geometries([source_temp, target_temp])
        source_temp.paint_uniform_color([1, 0.706, 0])
        target_temp.paint_uniform_color([0, 0.651, 0.929])
        source_temp.transform(self.icp_transformation)

        o3d.visualization.draw_geometries([source_temp, target_temp])
        self.transformed_pcd = source_temp


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

def final_noise_removal(final_object_model_):
    final_object_model = copy.deepcopy(final_object_model_)

    rgb = np.array(final_object_model.colors)
    idx = np.where(np.average(rgb, axis=1) > 0.2)[0]
    final_object_model = final_object_model.select_by_index(idx)


    cl, ind = final_object_model.remove_radius_outlier(nb_points=300, radius=5)
    final_object_model = final_object_model.select_by_index(ind)
    cl, ind = final_object_model.remove_radius_outlier(nb_points=200, radius=5)
    final_object_model = final_object_model.select_by_index(ind)
    cl, ind = final_object_model.remove_radius_outlier(nb_points=500, radius=10)
    final_object_model = final_object_model.select_by_index(ind)


    return final_object_model




class contruct_3d_model(ICP_class):
    def __init__(self, target_path, icp_threshold, mode, visualize = False, icp_visualize = False):
        target_pcd = o3d.io.read_point_cloud(target_path)
        # target_pcd.scale(1000, np.array([0., 0., 0.]))
        target_pcd = target_pcd.voxel_down_sample(voxel_size=0.5)
        target_component = segment_component(target_pcd)

        self.target_pcd = target_pcd
        self.target_hand = target_component.pcd_hand
        self.target_object = target_component.pcd_object
        self.threshold = icp_threshold
        self.visualize = visualize
        self.icp_visualize = icp_visualize
        self.mode = mode


    def stitch_pcd(self, source_path_list):

        for source_path in source_path_list:
            transformed_source_pcd = self._compensate_source_pcd_offset(source_path)

            if self.visualize:
                print("visualize stitched pcd")
                o3d.visualization.draw_geometries([self.target])
            self._update_target( self.target_pcd, transformed_source_pcd)

            if self.icp_visualize:
                super().icp_visualizer()

        custom_draw_geometry_with_rotation(copy.deepcopy(self.target).rotate([[1,0,0],[0,0,1],[0,-1,0]]))


    def temporal_coordinate_correction(self, pcd):
        trans_init = np.array([[-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])  ## TODO
        tno_temp = copy.deepcopy(pcd)
        tno_temp.transform(trans_init)

        return tno_temp

    def _compensate_source_pcd_offset(self, source_path):
        print(source_path)
        source_pcd = o3d.io.read_point_cloud(source_path)
        source_pcd = source_pcd.voxel_down_sample(voxel_size=0.5)
        source_component = segment_component(source_pcd)
        self.source_hand = source_component.pcd_hand
        self.source_object = source_component.pcd_object

        trans_init = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])  ## TODO
        # tno_temp = copy.deepcopy(self.source_hand)
        # tno_temp.transform(trans_init)

        if self.mode == 'hand':
            icp_tf = super().ICP(self.source_hand,self.target_hand, self.threshold,trans_init)
        elif self.mode == 'object':
            icp_tf = super().ICP(self.source_object,self.target_object, self.threshold, trans_init)
        elif self.mode == 'interest':
            icp_tf = super().ICP( self._add_two_pointcloud(self.source_object, self.source_hand) , self._add_two_pointcloud(self.target_object, self.target_hand), self.threshold, trans_init)
        else:
            print("Error: absent of ICP modes")

        source_pcd.transform(icp_tf)


        if self.visualize:
            super().draw_registration_result()
        return source_pcd

    def _add_two_pointcloud(self, pcd1, pcd2):
        p1_color = np.asarray(pcd1.colors)
        p2_color = np.asarray(pcd2.colors)
        new_pcd_color = np.concatenate((p1_color, p2_color), axis=0)

        p1_point = np.asarray(pcd1.points)
        p2_point = np.asarray(pcd2.points)
        new_pcd_point = np.concatenate((p1_point, p2_point), axis=0)

        new_pcd = o3d.geometry.PointCloud()
        new_pcd.points = o3d.utility.Vector3dVector(new_pcd_point)
        new_pcd.colors = o3d.utility.Vector3dVector(new_pcd_color)

        return new_pcd



    def _update_target(self, pcd1, pcd2):
        new_target = self._add_two_pointcloud(pcd1, pcd2)
        target_component = segment_component(new_target)
        self.target_pcd = new_target
        self.target_hand = target_component.pcd_hand
        self.target_object = target_component.pcd_object


    def write_stitched_pcd(self, save_dir, type):
        final_model = segment_component(self.target_pcd)
        final_object_model = final_model.pcd_object

        if type == 'obj_w_hand':
            final_object_model = self._add_two_pointcloud(final_model.pcd_object, final_model.pcd_hand)
        if type == 'hand':
            final_object_model = final_model.pcd_hand

        final_object_model = final_object_model.voxel_down_sample(voxel_size=0.5)
        final_object_model = final_noise_removal(final_object_model)
        final_object_model = final_object_model.scale(0.001, np.array([0., 0., 0.]))  # [mm] scale to [m]

        # final_object_model = self.temporal_coordinate_correction(final_object_model ) ## TODO : erase this with better nominal data

        print( np.mean( np.array(final_object_model.points), axis=0) )
        # final_object_model = self.target_pcd.scale(0.001, np.array([0., 0., 0.]))  # [mm] scale to [m]
        o3d.io.write_point_cloud(save_dir, final_object_model)





class coordinate_postprocessing(ICP_class):
    def __init__(self, object_model_path, icp_threshold = 100, visualize= None, task_nominal_path = None):
        ## Standard Coordinate
        target_pcd = o3d.io.read_point_cloud(object_model_path)
        target_pcd.scale(1000, np.array([0., 0., 0.]))
        target_pcd = target_pcd.voxel_down_sample(voxel_size=0.5)
        target_pcd = final_noise_removal(target_pcd)

        target_segment = segment_component(target_pcd)
        self.target_hand = target_segment.pcd_hand
        self.target_object =  target_segment.pcd_object
        self.threshold = icp_threshold

        ####### REMOVE ######
        # task_nominal_pcd = o3d.io.read_point_cloud(task_nominal_path)
        # task_nominal_pcd = task_nominal_pcd .voxel_down_sample(voxel_size=0.5)
        # self.task_nominal_palm = self.palm(task_nominal_pcd)
        # self.task_nominal_object = self.object(task_nominal_pcd)

        #####################

        self.visualize = visualize
        # self.off_trans = self.calculate_offset_trans()
        self.pose = None
        self.pcd = None
        self.palm_pcd = None
        self.object_pcd = None
        self.threshold = icp_threshold

    # def calculate_offset_trans(self):
    #     trans_init = np.array([[-1,0,0,0],[0,-1,0,0],[0,0,1,0],[0,0,0,1]]) ## TODO
    #     tno_temp = copy.deepcopy(self.task_nominal_object)
    #     tno_temp.transform(trans_init)
    #
    #     # _icp = ICP(tno_temp, self.target_object , self.threshold)
    #     _icp = ICP_class(self.task_nominal_object, self.target_object, self.threshold, trans_init )
    #
    #     if self.visualize:
    #         _icp.draw_registration_result()
    #     return _icp.icp_transformation


    def segment(self, pc_path, voxel_size = 0.5):
        '''
        Input: pointcloud directory
        :return: segmented pointcloud
        '''

        print(pc_path)
        pcd = o3d.io.read_point_cloud(pc_path)
        pcd = pcd.voxel_down_sample(voxel_size=voxel_size)


        source_segment = segment_component(pcd)
        self.pcd = pcd
        self.source_hand = source_segment.pcd_hand
        self.source_object = source_segment.pcd_object
        # o3d.visualization.draw_geometries([self.task_nominal_object, self.task_nominal_palm, self.palm_pcd , self.object_pcd])

        # object_pcd = crop_boundingbox(copy.deepcopy(pcd), lower_bound,upper_bound )


    def icp(self):
        # _icp = ICP_class(self.palm_pcd, self.task_nominal_palm , 200)
        # _icp = ICP_class(self.object_pcd, self.target_object, self.threshold)

        # icp_tf = super().ICP(self.object_pcd, self.target_object, self.threshold)
        icp_tf = super().ICP(self.source_hand, self.target_hand, self.threshold)
        self.pose = icp_tf

        if self.visualize:
            super().draw_registration_result()

    def _add_two_pointcloud(self, pcd1, pcd2):
        p1_color = np.asarray(pcd1.colors)
        p2_color = np.asarray(pcd2.colors)
        new_pcd_color = np.concatenate((p1_color, p2_color), axis=0)

        p1_point = np.asarray(pcd1.points)
        p2_point = np.asarray(pcd2.points)
        new_pcd_point = np.concatenate((p1_point, p2_point), axis=0)

        new_pcd = o3d.geometry.PointCloud()
        new_pcd.points = o3d.utility.Vector3dVector(new_pcd_point)
        new_pcd.colors = o3d.utility.Vector3dVector(new_pcd_color)

        return new_pcd

    def save_result(self, filename):
        self.pcd.transform(self.pose)
        # self.pcd.transform(self.off_trans)
        self.pcd.scale(0.001, np.array([0.,0.,0.]))
        # o3d.io.write_point_cloud(filename, self.pcd )

        final_object_model = copy.deepcopy(self.source_object)
        # target_save.transform(self.pose).scale(0.001, np.array([0.,0.,0.]))

        # target_save_ = copy.deepcopy(self.source_hand)
        # target_save_.transform(self.pose).scale(0.001, np.array([0.,0.,0.]))



        # final_object_model = final_noise_removal(target_save)
        cl, ind = final_object_model.remove_radius_outlier(nb_points=100, radius=5)
        final_object_model = final_object_model.select_by_index(ind)
        final_object_model.transform(self.pose)



        # final_object_model = final_noise_removal( self._add_two_pointcloud(target_save_,target_save))
        final_object_model = final_object_model.scale(0.001, np.array([0.,0.,0.]))

        o3d.io.write_point_cloud(filename,final_object_model  )


