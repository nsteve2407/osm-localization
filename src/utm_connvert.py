#! /usr/bin/python3
import rospy
import utm
from geometry_msgs.msg import PoseArray,Pose

class converter:
    def __init__(self) :
        self.pub = rospy.Publisher("/pose_array",PoseArray,queue_size=10)
        self.op_pose = Pose()
        self.op_msg = PoseArray()
    
    def cb(self,msg):
        self.op_msg = PoseArray()

        for pose in msg.poses:
            lat, lon = utm.to_latlon(pose.position.x,pose.position.y,14,'R')
            self.op_pose = Pose()
            self.op_pose.position.x = lon
            self.op_pose.position.y = lat
            self.op_pose.position.z = 0
            self.op_pose.orientation = pose.orientation
            self.op_msg.poses.append(self.op_pose)
        self.op_msg.header = msg.header
        
        self.pub.publish(self.op_msg)

    










rospy.init_node("utm_to_latlon",anonymous=True)
conv = converter()
sub = rospy.Subscriber("/osm_pose_estimate",PoseArray,conv.cb)

rospy.spin()

                
