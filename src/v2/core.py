from cmath import atan, pi
from dis import dis
from math import atan2
from turtle import width
import numpy as np
import rospy
import ros_numpy
import pandas as pd
from scipy import ndimage
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
        self.map_nodes = np.where(self.base_map<1,1,0)
        self.nodes = np.transpose(np.nonzero(self.base_map<=1))
        self.map_view_df=None

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


    def SliceAtAngle(self,idx,idy,phi,base_map):
        
        x_l = idx - int(self.bev_img_width)
        x_h = idx + int(self.bev_img_width)
        y_l = idy - int(2*self.bev_img_width)
        y_h = idy + int(2*self.bev_img_width)

        if x_l>=0 and x_h<self.map_bev.shape[0] and y_l>=0 and y_h<self.map_bev.shape[1]:
            img = base_map[x_l:x_h,y_l:y_h]
            img = ndimage.rotate(img,-np.rad2deg(phi),reshape=False)
            img = img[int(img.shape[0]/2):int(img.shape[0]/2+self.bev_img_width),int(img.shape[1]/2-self.bev_img_width):int(img.shape[1]/2+self.bev_img_width)]
            return img
        else:
            return np.zeros((self.bev_img_width,2*self.bev_img_width),dtype=np.uint8)

    def findRoadOrientation(self):
        poses = []
        road_nodes = self.nodes.tolist()
        road_nodes.sort()

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

    def isWithinBounds(self,idx,idy,mat):
        if idx>=0 and idx<mat.shape[0] and idy>=0 and idy<mat.shape[1]:
            return True
        else:
            return False

    def findNearestOne(self,idx,idy,mat,offset):
        visited = np.zeros_like(mat)
        visited[idx,idy]=1
        q = []
        if self.isWithinBounds(idx-offset,idy,mat): q.append([idx-offset,idy])
        if self.isWithinBounds(idx+offset,idy,mat): q.append([idx+offset,idy])
        if self.isWithinBounds(idx,idy-offset,mat): q.append([idx,idy-offset])
        if self.isWithinBounds(idx,idy+offset,mat): q.append([idx,idy+offset])

        while len(q)!=0:
            x,y = q.pop(0)
            if visited[x,y]==0:
                visited[x,y]=1
                if mat[x,y]==1:
                    return x,y
                else:
                    if self.isWithinBounds(x-1,y,mat) and abs((x-1)-idx)>offset and visited[x-1,y]==0: q.append([x-1,y])
                    if self.isWithinBounds(x+1,y,mat) and abs((x+1)-idx)>offset and visited[x+1,y]==0 : q.append([x+1,y])
                    if self.isWithinBounds(x,y-1,mat) and abs((y-1)-idy)>offset and visited[x,y-1]==0 : q.append([x,y-1])
                    if self.isWithinBounds(x,y+1,mat) and abs((y+1)-idy)>offset and visited[x,y+1]==0 : q.append([x,y+1])
        
        return -1,-1

    def findNearestOne2(self,idx,idy,d_tol):
        dist = np.abs(self.nodes-np.array([idx,idy]))
        dist = np.sum(dist,axis=1)
        dist_sorted = np.sort(dist)
        d = dist_sorted[int(np.argwhere(dist_sorted>d_tol)[0])]
        i = int(np.argwhere(dist==d)[0])
        
        return self.nodes[i,0],self.nodes[i,1]
        


    def findRoadOrientation2(self):
        poses = []
        d_tol=200
        road_nodes = self.nodes.tolist()
        offset = 2

        for i in range(len(road_nodes)):

            ix,iy = road_nodes[i][0],road_nodes[i][1]
            # sx_l = 0 if ix-search_space<0 else ix-search_space
            # sx_u = self.map_nodes.shape[0]-1 if ix+search_space>=self.map_nodes.shape[0] else ix+search_space
            # sy_l = 0 if iy-search_space<0 else iy-search_space
            # sy_u = self.map_nodes.shape[1]-1 if iy+search_space>=self.map_nodes.shape[1] else iy+search_space
            # mat = self.map_nodes[sx_l:sx_u,sy_l:sy_u]
            # x2,y2 = self.findNearestOne(ix,iy,self.map_nodes,offset)
            x2,y2 = self.findNearestOne2(ix,iy,d_tol)
            if x2!=-1 and y2!=-1:
                # angle = np.real(atan((y2-iy)/(x2-ix)))
                ix,iy = self.idx2coord(ix,iy)
                x2,y2 = self.idx2coord(x2,y2)
                angle = atan2((y2-iy),(x2-ix))
                poses.append(road_nodes[i]+[angle])
                poses.append(road_nodes[i]+[angle+np.pi])

            


        return poses
    
    def initMap(self):
        poses = self.findRoadOrientation()
        mapviews = []

        for pose in poses:
            mapView = self.SliceAtAngle(pose[0],pose[1],pose[2],self.map_bev)
            nodemap = self.SliceAtAngle(pose[0],pose[1],pose[2],self.map_nodes)
            norm = np.linalg.norm(mapView)
            if norm**2>=1.0:
                pose.append(mapView)
                pose.append(nodemap)             
                pose.append(norm**2)
                mapviews.append(pose)

        self.map_view_df = pd.DataFrame(mapviews,columns=['idx','idy','phi','image','nodemap','norm'])


