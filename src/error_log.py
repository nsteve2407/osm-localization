#! /usr/bin/env python3
import rospy as rp
import geodesy.utm as utm
from geometry_msgs.msg import PoseStamped,Vector3Stamped,PointStamped
from sensor_msgs.msg import NavSatFix
import pandas as pd
import tf_conversions as tf
import message_filters as mf
import numpy as np

class error_log:
    def __init__(self) -> None:
        self.df = []
        self.gps_n =[]
        self.gps_e = []
        self.pf_n = []
        self.pf_e = []
        self.pf_theta = []
        self.count=0
        self.log =False
        self.gps_pub = rp.Publisher("gps_utm",PointStamped,queue_size=100)
        


    def cb(self,gps_msg,pf_msg):
        gps_lat,gps_lon,gps_alt = gps_msg.latitude,gps_msg.longitude,gps_msg.altitude
        utm_p = utm.fromLatLong(gps_lat,gps_lon,gps_alt)
        self.gps_e.append(utm_p.easting)
        self.gps_n.append(utm_p.northing)
        self.pf_e.append(pf_msg.pose.position.x)
        self.pf_n.append(pf_msg.pose.position.y)
        q = (pf_msg.pose.orientation.x,pf_msg.pose.orientation.y,pf_msg.pose.orientation.z,pf_msg.pose.orientation.w)
        self.pf_theta.append(tf.transformations.euler_from_quaternion(q)[2])
        self.count+=1
    

    def cb2(self,gps_msg,pf_msg):
        gps_lat,gps_lon,gps_alt = gps_msg.vector.x,gps_msg.vector.y,gps_msg.vector.z
        utm_p = utm.fromLatLong(gps_lat,gps_lon,gps_alt)
        self.gps_e.append(utm_p.easting)
        self.gps_n.append(utm_p.northing)
        self.pf_e.append(pf_msg.pose.position.x)
        self.pf_n.append(pf_msg.pose.position.y)
        q = (pf_msg.pose.orientation.x,pf_msg.pose.orientation.y,pf_msg.pose.orientation.z,pf_msg.pose.orientation.w)
        self.pf_theta.append(tf.transformations.euler_from_quaternion(q)[2])
        self.count+=1

        point = PointStamped()
        point.point.x = utm_p.easting
        point.point.y = utm_p.northing
        point.header  = gps_msg.header
        point.header.frame_id = pf_msg.header.frame_id
        self.gps_pub.publish(point)


    def __del__(self):
        print("Logged {} messages".format(self.count))
        vals= {'gps_e':self.gps_e,'gps_n': self.gps_n,'pf_e':self.pf_e,'pf_n':self.pf_n,'pf_theta':self.pf_theta}
        self.df = pd.DataFrame(vals)
        # self.df.to_csv('./logs_no_error.csv')
        self.df['diff_e'] = self.df['gps_e']-self.df['pf_e']
        self.df['diff_e']  = self.df['diff_e']**2
        self.df['diff_n'] = self.df['gps_n']-self.df['pf_n']
        self.df['diff_n'] = self.df['diff_n']**2
        self.df['diff_sum'] = self.df['diff_e']+self.df['diff_n']
        self.df['error'] = np.sqrt(self.df['diff_sum'])
        

        if self.log:
            self.df.to_csv('./u_calib_local_quadratic.csv')
            print('\nLog file saved !\nExiting..')

log = error_log()

# def main():
#     rp.init_node('localization_error_log',anonymous=True)
    
#     gps_sub = mf.Subscriber('/vehicle/gps/fix',NavSatFix)
#     pf_sub = mf.Subscriber('/osm_average_pose_estimate',PoseStamped)

#     ats = mf.ApproximateTimeSynchronizer([gps_sub,pf_sub],50000,0.5)
#     ats.registerCallback(log.cb)


#     rp.spin()


def main():
    rp.init_node('localization_error_log',anonymous=True)
    
    gps_sub = mf.Subscriber('/filter/positionlla',Vector3Stamped)
    pf_sub = mf.Subscriber('/osm_average_pose_estimate',PoseStamped)

    ats = mf.ApproximateTimeSynchronizer([gps_sub,pf_sub],50000,0.1)
    ats.registerCallback(log.cb2)


    rp.spin()


if __name__ == "__main__":
    main()

    if rp.is_shutdown():
        del log


