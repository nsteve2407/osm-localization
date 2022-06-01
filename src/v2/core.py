from cmath import atan, pi
from turtle import width
import numpy as np
import rospy
import ros_numpy
import pandas as pd

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

        self.base_map = np.squeeze(np.load(path_to_mat),axis=-1)
        self.map_bev = np.where(self.base_map<road_width,1,0)
        # self.map_nodes = np.where(self.base_map<1,1,0)
        self.nodes = np.transpose(np.nonzero(self.base_map<=1))

    def bev_xy2index_lidar(self,x,y):
        if y>0.0:
            idy = int(self.bev_img_width-int((y/self.res_y)))
        else:
            idy = int(self.bev_img_width+int(abs(y)/self.res_y))
        
        idx = self.bev_img_width-int(x/self.res_x)
        return idx-1,idy-1

    def pointcloud2grid(self,pc_msg):
        
        bev_image  = np.zeros((self.bev_img_width,2*self.bev_img_width),dtype=np.uint8)
        pc_array = ros_numpy.numpify(pc_msg)
        idx = np.transpose(np.nonzero(pc_array['intensity']==255.0))
        points = pc_array[idx[:,0],idx[:,1]]

        for i in range(points.shape[0]):
            ix,iy=self.bev_xy2index_lidar(points[i]['x'],points[i]['y'])
            if ix<bev_image.shape[0] and iy<bev_image.shape[1] and ix>=0 and iy>=0:
                bev_image[ix,iy]=1
        
        return bev_image

    def idx2coord(self,idx,idy):
        return self.origin_y+idy*self.res_y,self.origin_x+idx*self.res_x

    def coord2idx(self,e,n):
        return int(np.round((n-self.origin_y)/self.res_y))-1,int(np.round((e-self.origin_x)/self.res_x))-1


    def SliceAtAngle(self,idx,idy,phi):
        return 0

    def findRoadOrientation(self):
        poses = []
        road_nodes = self.nodes.tolist()

        for i in range(len(road_nodes)):

            if(i!=0 and i<(len(road_nodes)-1)):
                x1,y1 = road_nodes[i-1]
                x2,y2 = road_nodes[i+1]
                x1,y1 = self.idx2coord(x1,y1)
                x2,y2 = self.idx2coord(x1,y1)
                angle = np.real(atan((y2-y1)/(x2-x1)))
                poses.append(road_nodes[i]+[angle])
                poses.append(road_nodes[i]+[angle+np.pi])               
            elif (i==0):
                x1,y1 = road_nodes[i]
                x2,y2 = road_nodes[i+1]
                x1,y1 = self.idx2coord(x1,y1)
                x2,y2 = self.idx2coord(x1,y1)
                angle =  np.real(atan((y2-y1)/(x2-x1)))
                poses.append(road_nodes[i]+[angle])
                poses.append(road_nodes[i]+[angle+np.pi])

            else:
                x1,y1 = road_nodes[i-1]
                x2,y2 = road_nodes[i]
                x1,y1 = self.idx2coord(x1,y1)
                x2,y2 = self.idx2coord(x1,y1)
                angle = np.real(atan((y2-y1)/(x2-x1)))
                poses.append(road_nodes[i]+[angle])
                poses.append(road_nodes[i]+[angle+np.pi])




        return poses
    


