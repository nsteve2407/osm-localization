#include "include/osm_pf.h"
#include<random>
#include<math.h>
#include<tf/transform_datatypes.h>
#include<pcl_ros/transforms.h>
#include<Eigen/Core>
#include<pcl_conversions/pcl_conversions.h>
#include<geometry_msgs/PoseArray.h>
using namespace osmpf;

osm_pf::osm_pf(std::string path_to_d_mat,f min_x,f min_y,f Max_x,f Max_y,int particles)
{
    num_particles = particles;
    d_matrix = xt::load_npy<f>(path_to_d_mat);
    origin_x = min_x;
    origin_y = min_y;
    max_x = Max_x;
    max_y = Max_y;
    pf_publisher = nh.advertise<geometry_msgs::PoseArray>("oms_pose_estimate",100);
    odom_sub.subscribe(nh,"odometry_topic",100);
    pc_sub.subscribe(nh,"os_cloud_node/points",100);
    sync = new message_filters::Synchronizer<sync_policy>(sync_policy(10),odom_sub,pc_sub);
    cov_lin = (f)25;
    cov_angular = (f)M_PI_2;
    // Xt = std::make_shared<std::vector<pose>>();
    // Wt = std::make_shared<std::vector<f>>();
    Xt = std::vector<pose>(num_particles);
    Wt = std::vector<f>(num_particles);
    init_particles();
}

void osm_pf::init_particles()
{
    std::default_random_engine generator;
    std::uniform_real_distribution<f> distribution_x(origin_x,max_x);
    std::uniform_real_distribution<f> distribution_y(origin_y,max_y);
    std::uniform_real_distribution<f> distribution_theta(0,(f)M_PI*2);

    for(int i=0;i<num_particles;i++)
    {
        f _x = distribution_x(generator);
        f _y = distribution_y(generator);
        f _theta = distribution_theta(generator);
        pose p(_x,_y,_theta);
        Xt[i]=p;
    }    
}

pose osm_pf::find_xbar(pose x_tminus1,nav_msgs::Odometry odom)
{
    f dx = odom.pose.pose.position.x;
    f dy = odom.pose.pose.position.y;
    f dtheta =  tf::getYaw(odom.pose.pose.orientation); //check sign converntions

    std::default_random_engine generator;
    std::normal_distribution<f> dist_x(x_tminus1.x+dx,cov_lin);
    std::normal_distribution<f> dist_y(x_tminus1.y+dy,cov_lin);
    std::normal_distribution<f> dist_theta(x_tminus1.theta+dtheta,cov_angular);
    pose p_k_bar(dist_x(generator),dist_y(generator),dist_theta(generator));
    return p_k_bar;
}

std::vector<pose> osm_pf::find_Xbar(std::vector<pose> X_tminus1,nav_msgs::Odometry odom)
{
    std::vector<pose> Xt_bar(num_particles);
    for(int i=0;i<num_particles;i++)
    {
        Xt_bar[i] = find_xbar(X_tminus1[i],odom);
    }
    return Xt_bar;

}

osm_pf::f osm_pf::find_wt_point(pcl::PointXYZI point)
{
    auto e = point.x - origin_x;
    auto n = point.y - origin_y;
    auto i = point.intensity;
    f wt,distance,r_w;
    r_w = (f)8.0;

    if(point.x>origin_x && point.x<max_x && point.y<origin_y && point.y>max_y)
    {
        distance = d_matrix(n,e);
    }
    else
    {
        distance = -1.0;
    }

    if(distance>0.0)
    {
        wt = 1/(1+exp(distance-r_w));
        if(i != 255.0)
        {
            wt = (f)1 - wt;
        }
    }
    else
    {
        wt =1.0;
    }
    



    return wt;
    
}

osm_pf::f osm_pf::find_wt(pose xbar,sensor_msgs::PointCloud2 p_cloud)
{
    Eigen::Matrix4f T;
    T<<cos(xbar.theta), -sin(xbar.theta),0, xbar.x,
       sin(xbar.theta), cos(xbar.theta), 0, xbar.y,
       0, 0, 1,0,
       0,0,0,1;
    
    sensor_msgs::PointCloud2 p_cloud_mapFrame;
    pcl_ros::transformPointCloud(T,p_cloud,p_cloud_mapFrame);
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;

    pcl::fromROSMsg(p_cloud_mapFrame,pcl_cloud);
    
    f weight = 1;
    for(auto x : pcl_cloud.points)
    {
        weight = weight*find_wt_point(x);
    }

    return weight;


}

std::vector<osm_pf::f> osm_pf::find_Wt(std::vector<pose> Xtbar,sensor_msgs::PointCloud2 p_cloud)
{
    std::vector<f> weights;
    f weight;
    for(pose p: Xtbar)
    {
        weight=find_wt(p,p_cloud);
        weights.push_back(weight);
    }

    return weights;
}

std::vector<pose> osm_pf::sample_xt(std::vector<pose> Xbar_t,std::vector<f> Wt)
{
    std::vector<pose> Xt;

    std::default_random_engine generator;
    std::discrete_distribution<f> distribution;

    for(int i=0;i<Xbar_t.size();i++)
    {
        int idx = distribution(generator);
        Xt.push_back(Xbar_t[idx]);
    }

    return Xt;
}

void osm_pf::callback(nav_msgs::Odometry u,sensor_msgs::PointCloud2 z)
{
    
    std::vector<pose> Xbar = find_Xbar(Xt,u);
    std::vector<f> Wt_est  = find_Wt(Xbar,z);
    std::vector<pose> X_t_est = sample_xt(Xbar,Wt);
    Xt = X_t_est;
    Wt = Wt_est;

    geometry_msgs::PoseArray msg;
    geometry_msgs::Quaternion q;
    geometry_msgs::Point position;
    geometry_msgs::Pose p;
    for(pose x: X_t_est)
    {
        q = tf::createQuaternionMsgFromYaw(x.theta);
        position.x = x.x;
        position.y = x.y;
        position.z = 0.0;
        p.position = position;
        p.orientation = q;
        msg.poses.push_back(p);  
    }

    pf_publisher.publish(msg);    
}

void osm_pf::run()
{
    sync->registerCallback(boost::bind(&callback,_1,_2));
}