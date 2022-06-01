from turtle import width
import numpy as np
import rospy
import ros_numpy

class osm_v2():
    def __init__(self):
        path_to_mat = rospy.get_param("/osm_particle_filter/path_to_dmat")
        road_width =  rospy.get_param("/osm_particle_filter/road_width")
        self.res_x = rospy.get_param("/osm_particle_filter/map_resolution_x")
        self.res_y = rospy.get_param("/osm_particle_filter/map_resolution_y")
        self.origin_x = rospy.get_param('/osm_particle_filter/min_x')
        self.origin_y = rospy.get_param('/osm_particle_filter/min_y')
        self.sensor_range = rospy.get_param('/osm_particle_filter/sensor_range')
        self.bev_img_width = int(self.sensor_range/self.res_x)

        self.base_map = np.load(path_to_mat)
        self.map_bev = np.where(self.base_map<road_width,1,0)
        # self.map_nodes = np.where(self.base_map<1,1,0)
        self.nodes = np.transpose(np.nonzero(self.base_map<=1))

    def bev_xy2index_lidar(self,x,y):
        if x>0.0:
            idx=width/2-int((x/self.res_x))
        else:
            idx=width/2+int(abs(x)/self.res_x)
        
        idy = int(y/self.res_y)
        return idx,idy

    def pointcloud2grid(self,pc_msg):
        
        bev_image  = np.zeros((self.bev_img_width,2*self.bev_img_width),dtype=np.int)
        pc_array = ros_numpy.numpyify(pc_msg)
        idx = np.transpose(np.nonzero(pc_array['intensity']==255.0))
        points = pc_array(idx[:,0],idx[:,1])

        for pt in points:
            bev_image[self.bev_xy2index_lidar(pt['x'],pt['y'])]=1
        
        return bev_image




