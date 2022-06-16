from cmath import atan, pi
from dis import dis
from math import atan2
from mimetypes import guess_all_extensions
from turtle import width
from cv2 import getGaussianKernel
import numpy as np
import rospy
import ros_numpy
import pandas as pd
from scipy import ndimage
class osm_v2():
    def __init__(self,load_saved_df=False):
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
        
        self.ranges=np.arange(2,15).astype(np.int32)
        if load_saved_df:
            path = rospy.get_param('/osm_particle_filter/path_to_map_df')
            self.map_view_df= pd.read_pickle(path)

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
        
        x_l = idx - int(2*self.bev_img_width)
        x_h = idx + int(2*self.bev_img_width)
        y_l = idy - int(4*self.bev_img_width)
        y_h = idy + int(4*self.bev_img_width)

        if x_l>=0 and x_h<self.map_bev.shape[0] and y_l>=0 and y_h<self.map_bev.shape[1]:
            img = base_map[x_l:x_h,y_l:y_h]
            img = ndimage.rotate(img,-np.rad2deg(phi),reshape=False)
            img = img[int(img.shape[0]/2):int(img.shape[0]/2+self.bev_img_width),int(img.shape[1]/2-self.bev_img_width):int(img.shape[1]/2+self.bev_img_width)]
            return img
        else:
            return np.zeros((self.bev_img_width,2*self.bev_img_width),dtype=np.uint8)

    def SliceNoRotation(self,idx,idy):
        x_l = idx - int(self.bev_img_width/2)
        x_h = x_l + int(self.bev_img_width)
        y_l = idy - int(self.bev_img_width)
        y_h = y_l + int(2*self.bev_img_width)
        if x_l>=0 and x_h<self.map_bev.shape[0] and y_l>=0 and y_h<self.map_bev.shape[1]:
            return self.map_bev[x_l:x_h,y_l:y_h]
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
        d_tol=10
        road_nodes = self.nodes.tolist()
        offset = 4

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
                angle = np.real(atan((y2-iy)/(x2-ix+0.000000001)))
                # ix,iy = self.idx2coord(ix,iy)
                # x2,y2 = self.idx2coord(x2,y2)
                # angle = atan2((y2-iy),(x2-ix))
                poses.append(road_nodes[i]+[angle])
                poses.append(road_nodes[i]+[angle+np.pi])

            


        return poses
    
    def initMap(self):
        poses = self.findRoadOrientation2()
        mapviews = []

        for pose in poses:
            mapView = self.SliceAtAngle(pose[0],pose[1],pose[2],self.map_bev)
            nodemap = self.SliceAtAngle(pose[0],pose[1],pose[2],self.map_nodes)
            pixel_count = (np.linalg.norm(mapView))**2
            if pixel_count>=1.0:
                # pose.append(mapView)
                # pose.append(nodemap)             
                pose.append(pixel_count)
                mapviews.append(pose)

        # self.map_view_df = pd.DataFrame(mapviews,columns=['idx','idy','phi','image','nodemap','norm'])
        self.map_view_df = pd.DataFrame(mapviews,columns=['idx','idy','phi','pixel_count'])

    def initMap_noRotation(self):
        poses = self.nodes.tolist()
        mapviews = []
        for pose in poses:
            mapView = self.SliceNoRotation(pose[0],pose[1])
            # nodemap = self.SliceAtAngle(pose[0],pose[1],pose[2],self.map_nodes)
            pixel_count = (np.linalg.norm(mapView))**2
            if pixel_count>=1.0:
                # pose.append(mapView)
                # pose.append(nodemap)             
                e,n = self.idx2coord(pose[0],pose[1])
                pose.append(e)
                pose.append(n)
                pose.append(pixel_count)
                mapviews.append(pose)

        # self.map_view_df = pd.DataFrame(mapviews,columns=['idx','idy','phi','image','nodemap','norm'])
        self.map_view_df = pd.DataFrame(mapviews,columns=['idx','idy','e','n','pixel_count'])

    def initMapDescriptors(self):
        poses = self.nodes.tolist()
        mapviews = []
        
        for pose in poses:
            mapView = self.SliceNoRotation(pose[0],pose[1])
            Desc  = self.findRoadDescriptor(mapView,self.ranges)
            D_1d = Desc.sum(axis=-1)
            pixel_count = (np.linalg.norm(mapView))**2
            if pixel_count>=1.0:
                # pose.append(mapView)
                # pose.append(nodemap)             
                e,n = self.idx2coord(pose[0],pose[1])
                pose.append(e)
                pose.append(n)
                pose.append(pixel_count)
                pose.append(Desc)
                pose.append(D_1d)
                mapviews.append(pose)

        self.map_view_df = pd.DataFrame(mapviews,columns=['idx','idy','e','n','pixel_count','descriptor2d','descriptor1d'])

    def findGolbaltopX(self,scanImage,X):
        query_pixel_count = (np.linalg.norm(scanImage))**2
        self.map_view_df['similarity']=(1-(np.abs(self.map_view_df['pixel_count']-query_pixel_count)/(query_pixel_count))).clip(0.0,None)
        # self.map_view_df['similarity']=np.abs(self.map_view_df['pixel_count']-query_pixel_count)
        self.map_view_df.sort_values(by='similarity',inplace=True,ascending=False)
        # self.map_view_df = self.map_view_df[self.map_view_df['similarity']>=0.65]

        return self.map_view_df.iloc[:X,:].copy()
        # return self.map_view_df

    def findGolbaltopX_descriptor(self,scanImage,X,init_guess=[0.0,0.0,0.0]):
        if init_guess[0]!=0.0 or init_guess[1]!=0.0 or init_guess[2]!=0.0:
            y,x,R=init_guess
            df = self.map_view_df[(self.map_view_df.e>x-R) & (self.map_view_df.e<x+R) & (self.map_view_df.n>y-R )& (self.map_view_df.n<y+R)]
        else:
            df = self.map_view_df.copy()
        query_pixel_count = (np.linalg.norm(scanImage))**2
        q_dec2d = self.findRoadDescriptor(scanImage,self.ranges)
        q_dec1d = q_dec2d.sum(axis=-1)
        df['diff1d']=df.descriptor1d.apply(lambda x:np.linalg.norm(x-q_dec1d))
        df['diffnorm']=np.abs(df['pixel_count']-query_pixel_count)
        df['f1'] = 2*(df.diff1d*df.diffnorm)/(df.diff1d+df.diffnorm)
        df.sort_values(by=['diff1d','diffnorm'],inplace=True,ascending=True)
        # df = df[df['similarity']>=0.65]

        return df.iloc[:X,:]
        # return self.map_view_df
    def findPose(self,scanImage,X,angular_res):
        top100 = self.findGolbaltopX(scanImage,X)
        countScanImage = np.linalg.norm(scanImage)**2
        poses = []
        for i in range(top100.shape[0]):
            x = top100.iloc[i,0]
            y = top100.iloc[i,1]
            p=[]
            max_score = 0.0
            for phi in np.arange(0,360,angular_res):
                img = self.SliceAtAngle(x,y,np.deg2rad(phi),self.map_bev)
                px_count = np.linalg.norm(img)**2
                overlap = np.sum(img*scanImage)/countScanImage
                similarity = max(0.0,1-np.abs(px_count-countScanImage)/countScanImage)
                f1score = (5*overlap*similarity)/(overlap+4*similarity)
                if f1score>max_score:
                    # similarity = top100.similarity[i]
                    # f1score = (2*overlap*similarity)/(overlap+similarity)
                    e,n = self.idx2coord(x,y)
                    # poses.append([x,y,e,n,np.deg2rad(phi),img,score])
                    p=[x,y,e,n,np.deg2rad(phi),f1score]
                    max_score=f1score
            if len(p)>0:
                poses.append(p)

        # df = pd.DataFrame([poses],columns=['idx','idy','e','n','phi','mapview','score'])
        df = pd.DataFrame(poses,columns=['idx','idy','e','n','phi','f1score'])
        df.sort_values(by='f1score',inplace=True,ascending=False)
        return df
    
    def findIndxAtRange(self,img,R):
        cx,cy = img.shape[0]/2, img.shape[1]/2
        f = lambda theta: [int(cx+((R*np.cos(theta))/self.res_x)),int(cy+((R*np.sin(theta))/self.res_y))]
        indxy = np.array([f(x) for x in np.deg2rad(range(360))])

        d = img[indxy[:,0],indxy[:,1]]
        return d

    def findRoadDescriptor(self,image,Range_list):
        d = np.array([self.findIndxAtRange(image,r) for r in Range_list])
        return d

