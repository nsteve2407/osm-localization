#! /usr/bin/env python3
import rospy 
from geometry_msgs.msg import Pose2D, PoseArray,Pose
from sensor_msgs.msg import Image
from core import *
from tf.transformations import quaternion_from_euler
import ros_numpy
from copy import deepcopy

class Road_Desc_Search(osm_v2):
    def __init__(self):
        super().__init__(True)
        self.seed_x = rospy.get_param('/osm_particle_filter/seed_x')
        self.seed_y = rospy.get_param('/osm_particle_filter/seed_y')
        self.ang_res = rospy.get_param('/osm_particle_filter/pose_angular_res')
        self.globaltopX = rospy.get_param('/osm_particle_filter/global_search_topX')
        self.localtopX = rospy.get_param('/osm_particle_filter/local_topX')
        self.publisher = rospy.Publisher('/osm_road_descriptor_poses',PoseArray,queue_size=100)
        self.lidar_bev_sub = rospy.Subscriber('/lidar_bev',Image,self.callback)
        self.range = rospy.get_param('/osm_particle_filter/init_cov_linear')

    def callback(self,lidar_bev_image):
        img  = ros_numpy.numpify(lidar_bev_image)/255
        query_desc_2d  = self.findRoadDescriptor(img,self.ranges)
        df = self.findGolbaltopX_descriptor(img,query_desc_2d,self.globaltopX,[self.seed_x,self.seed_y,self.range])
        df = self.find_descriptor_Pose(df,query_desc_2d,self.ang_res) #Pose search step
        p_array = []
        p = Pose()
        for i in range(self.localtopX):
            # p.append(Pose2D(df.e.iloc[i],df.n.iloc[i],0))
            p.position.x = df.e.iloc[i]
            p.position.y= df.n.iloc[i]
            p.position.z = 0.0
            q = quaternion_from_euler(0,0,df.phi.iloc[i])
            p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w = q[0],q[1],q[2],q[3]  
            p_array.append(deepcopy(p))          

     
        msg = PoseArray()
        msg.header.frame_id = 'map'
        msg.header.stamp = lidar_bev_image.header.stamp
        msg.poses = p_array
        self.publisher.publish(msg)
        

def main():
    rospy.init_node("osm_pose_generator")
    desc_search = Road_Desc_Search()

    while not rospy.is_shutdown():   
        rospy.spin()

if __name__=="__main__":
    main()