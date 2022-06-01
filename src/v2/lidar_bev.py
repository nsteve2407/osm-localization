from core import *
from sensor_msgs.msg import Image,PointCloud2




class bev_pub():
    def __init__(self):
        self.Core =  osm_v2()
        self.bev_pub = rospy.Publisher('/lidar_bev',Image,100)

    def cb(self,pc_msg):
        bev_img_ = self.Core.pointcloud2grid(pc_msg)*255.0

        img_msg = ros_numpy.msgify(Image,bev_img_)
        img_msg.header.stamp = pc_msg.header.stamp

        self.bev_pub.publish(img_msg)


bev = bev_pub()
road_points_topic = '/road_points'
rospy.init_node('lidar_bev',anonymous=True)
sub = rospy.Subscriber(road_points_topic,PointCloud2,bev.cb)

rospy.spin()