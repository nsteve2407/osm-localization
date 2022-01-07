#! /usr/bin/env python3
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from dbw_mkz_msgs.msg import WheelSpeedReport, SteeringReport
import rospy
import math
from tf.transformations import quaternion_from_euler
import message_filters as mf
# parameters
wheelbase = 2.84988
steer_ratio  = 14.8
wheel_rad  =0.33
beta=0.5
class odom:
    def __init__(self,b_size=1):
        self.buffer_size = b_size
        self.count = 0
        self.pub = rospy.Publisher("/vehicle/odometry",Odometry,queue_size=10)
        self.Vf =0.0
        self.Vr = 0.0
        self.Vcg=0.0
        self.Rr = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.w = 0.0
        self.Vx = 0.0
        self.Vy = 0.0
        self.dt= 0.0
        self.t0 = 0.0
        self.t1 = 0.0

    def callback(self,wheel_msg,steer_msg):
        ws_f = (wheel_msg.front_left + wheel_msg.front_right)/2
        ws_r = (wheel_msg.rear_left + wheel_msg.rear_right)/2
        self.Vf  =ws_f*wheel_rad
        self.Vr  = ws_r*wheel_rad
        delta  = steer_msg.steering_wheel_angle/steer_ratio
        if abs(delta)>0.001:
            Rr  = wheelbase/math.tan(delta)
            if delta>0:
                self.w = beta*self.w + (1-beta)*self.Vr/Rr
            else:
                self.w = self.Vr/Rr
            Rcg  = math.sqrt((Rr**2)+(wheelbase/2)**2)
            self.Vcg =Rcg*self.w
        else:
            self.Vcg  = (self.Vf+self.Vr)/2
            self.w =0


        if self.count==0:
            self.t0  = wheel_msg.header.stamp.secs
            self.count+=1
            return

        if self.count < self.buffer_size:
            # self.t0 = wheel_msg.header.stamp.secs
            self.count+=1
            return

        else:
            self.dt  = wheel_msg.header.stamp.secs - self.t0
            self.theta += self.w*self.dt


            self.Vx  = self.Vcg*math.cos(self.theta)
            self.Vy  = self.Vcg*math.sin(self.theta)

            self.x += self.Vx*self.dt
            self.y += self.Vy*self.dt

            self.count = 0
            msg  = Odometry()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id  = 'odom'
            msg.child_frame_id = "base_footprint"
            msg.pose.pose.position.x = self.x
            msg.pose.pose.position.y  =self.y
            msg.pose.pose.position.z  = 0.0
            q  = quaternion_from_euler(0,0,self.theta)
            msg.pose.pose.orientation.x  = q[0]
            msg.pose.pose.orientation.y  = q[1]
            msg.pose.pose.orientation.z  = q[2]
            msg.pose.pose.orientation.w  = q[3]
            msg.pose.covariance  = [1,0,0,0,0,0, 0,1,0,0,0,0, 0,0,1,0,0,0, 0,0,0,1,0,0, 0,0,0,0,1,0, 0,0,0,0,0,1]

            msg.twist.twist.linear.x  = self.Vx
            msg.twist.twist.linear.y  = self.Vy
            msg.twist.twist.linear.z  = 0.0
            msg.twist.twist.angular.x  = 0.0
            msg.twist.twist.angular.y  = 0.0
            msg.twist.twist.angular.z  = self.w
            msg.twist.covariance = [1,0,0,0,0,0, 0,1,0,0,0,0, 0,0,1,0,0,0, 0,0,0,1,0,0, 0,0,0,0,1,0, 0,0,0,0,0,1]


            self.pub.publish(msg)
            

rospy.init_node('Odometry',anonymous=True)          
                
odometry  = odom()
wheel_sub  = mf.Subscriber('/vehicle/wheel_speed_report',WheelSpeedReport)
steering_sub  = mf.Subscriber('/vehicle/steering_report',SteeringReport)

ats  = mf.ApproximateTimeSynchronizer([wheel_sub,steering_sub],10,slop=0.1)
ats.registerCallback(odometry.callback)

rospy.spin()
