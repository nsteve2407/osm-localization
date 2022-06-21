#! /usr/bin/env python3
import rospy 
from geometry_msgs.msg import Pose2D
from osm_localization.srv import GlobalSearch,GlobalSearchRequest,GlobalSearchResponse
from core import *
import ros_numpy

def main():
    rospy.init_node("osm_gloabal_search_service")
    global_search_topX = rospy.get_param('/osm_particle_filter/global_search_topX')
    load_saved_df=True
    if load_saved_df:
        search = osm_v2(load_saved_df)
    else:
        search.initMapDescriptors()
    
    def service_callback(req):
        # print(type(srv_msg))
        img = ros_numpy.numpify(req.lidar_bev_image)/255
        df = search.findGolbaltopX_descriptor(img,global_search_topX,[req.seed_x,req.seed_y,req.range])
        poses = []
        for i in range(df.shape[0]):
            poses.append(Pose2D(df.e.iloc[i],df.n.iloc[i],0.0))
     
        resp = GlobalSearchResponse()
        resp.matches = poses
        
        return resp

    s = rospy.Service("osm_global_search",GlobalSearch,service_callback)
    # rospy.loginfo("OSM Global search service Initialized. Waiting for request ..\n")
    
    rospy.spin()

if __name__=="__main__":
    main()