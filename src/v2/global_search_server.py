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
    
    def service_callback(request):
        img = ros_numpy.numpify(request.lidar_bev_image)
        df = search.findGolbaltopX_descriptor(img,global_search_topX,[request.seed_x,request.seed_y,request.range])
        poses = []
        Pose = Pose2D()
        for i in range(df.shape[0]):
            Pose.x = df.e[i]
            Pose.y = df.n[i]
            Pose.theta = 0
            poses.append(Pose)

        return GlobalSearchResponse(poses)

    s = rospy.Service("osm_global_search",GlobalSearch,service_callback)
    rospy.loginfo("OSM Global search service Initialized. Waiting for request ..\n")
    
    rospy.spin()

if __name__=="__main__":
    main()