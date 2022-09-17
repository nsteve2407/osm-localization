#! /usr/bin/env python3
import rospy 
from geometry_msgs.msg import Pose2D, PoseArray,Pose
from sensor_msgs.msg import Image,NavSatFix
from core import *
from tf.transformations import quaternion_from_euler
import ros_numpy
from copy import deepcopy
import osmnx as ox
import matplotlib.pyplot as plt
import message_filters as mf
from geodesy.utm import fromLatLong


g = ox.graph_from_point((30.695,-96.464),dist=1500,dist_type="bbox")
# g = ox.graph_from_point(( 30.690984,-96.46288  ),dist=1500,dist_type="bbox")


g = ox.project_graph(g)


class Road_Desc_Search(osm_v2):
    def __init__(self):
        super().__init__(True)
        self.seed_x = rospy.get_param('/osm_particle_filter/seed_x')
        self.seed_y = rospy.get_param('/osm_particle_filter/seed_y')
        self.ang_res = rospy.get_param('/osm_particle_filter/pose_angular_res')
        self.globaltopX = rospy.get_param('/osm_particle_filter/global_search_topX')
        self.localtopX = rospy.get_param('/osm_particle_filter/local_topX')
        self.publisher = rospy.Publisher('/osm_road_descriptor_poses',PoseArray,queue_size=100)
        self.lidar_bev_sub = mf.Subscriber('/lidar_bev',Image)
        self.gps_sub = mf.Subscriber('/vehicle/gps/fix',NavSatFix)
        self.ats = mf.ApproximateTimeSynchronizer([self.lidar_bev_sub,self.gps_sub],queue_size=1,slop=0.1)
        self.ats.registerCallback(self.callback)
        self.range = rospy.get_param('/osm_particle_filter/init_cov_linear')

    def callback(self,lidar_bev_image,gps_msg):
        img  = ros_numpy.numpify(lidar_bev_image)/255
        latlon = fromLatLong(gps_msg.latitude,gps_msg.longitude)
        query_desc_2d  = self.findRoadDescriptor(img,self.ranges)
        query_desc_2d_frontal  = self.frontal_desc_from_img(img,self.ranges)
        df = self.findGolbaltopX_descriptor(img,query_desc_2d,self.globaltopX,[self.seed_x,self.seed_y,self.range])
        df = self.find_descriptor_Pose_frontal(df,query_desc_2d_frontal,self.ang_res) #Pose search step
        p_array = []
        p = Pose()
        fig,ax=ox.plot.plot_graph(g,bgcolor='white',figsize=(40,24),edge_color='black',edge_linewidth=2,show=False, close=False)
        ax.scatter(latlon.easting,latlon.northing,c='red',marker='*',linewidths=15)
        for i in range(self.localtopX):
            # p.append(Pose2D(df.e.iloc[i],df.n.iloc[i],0))
            p.position.x = df.e.iloc[i]
            p.position.y= df.n.iloc[i]
            p.position.z = 0.0
            q = quaternion_from_euler(0,0,df.phi.iloc[i])
            p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w = q[0],q[1],q[2],q[3]  
            p_array.append(deepcopy(p))          
            ax.scatter(p.position.x,p.position.y,c='blue',marker='.',linewidths=15)
        
        
        fig.savefig('/home/mkz/catkin_ws/desc_search_figs/desc_search{}.png'.format(lidar_bev_image.header.stamp),dpi=150) 
        fig.clf()
        # msg = PoseArray()
        # msg.header.frame_id = 'map'
        # msg.header.stamp = rospy.Time.now()
        # msg.poses = p_array
        # self.publisher.publish(msg)
        

def main():
    rospy.init_node("osm_pose_generator")
    desc_search = Road_Desc_Search()

    while not rospy.is_shutdown():   
        rospy.spin()

if __name__=="__main__":
    main()