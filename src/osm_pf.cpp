#include "osm_pf.h"
#include<random>
#include<math.h>
#include<tf/transform_datatypes.h>
#include<pcl_ros/transforms.h>
#include<Eigen/Core>
#include<pcl_conversions/pcl_conversions.h>
#include<geometry_msgs/PoseArray.h>
#include<geometry_msgs/Quaternion.h>
#include<boost/bind.hpp>
#include <pcl/common/transforms.h>
#include<iostream>
#include<algorithm>
#include<pcl/filters/voxel_grid.h>
#include <functional>
using namespace osmpf;

template <typename T>
std::vector<T> operator+(const std::vector<T>& a, const std::vector<T>& b)
{
    assert(a.size() == b.size());

    std::vector<T> result;
    result.reserve(a.size());

    std::transform(a.begin(), a.end(), b.begin(), 
                   std::back_inserter(result), std::plus<T>());
    return result;
}

osm_pf::osm_pf(std::string path_to_d_mat,f min_x,f min_y,f Max_x,f Max_y,f map_res,int particles,f seed_x,f seed_y)
{
    num_particles = particles;
    d_matrix = xt::load_npy<f>(path_to_d_mat);
    count = 0;
    origin_x = min_x;
    origin_y = min_y;
    max_x = Max_x;
    max_y = Max_y;
    pf_publisher = nh.advertise<geometry_msgs::PoseArray>("osm_pose_estimate",100);
    odom_sub.subscribe(nh,"/odometry/filtered",25);
    pc_sub.subscribe(nh,"/road_points",100);
    // pc_sub.subscribe(nh,"/road_points",100);
    sync.reset(new Sync(sync_policy(10),odom_sub,pc_sub)) ;
    cov_lin = (f)0.01;
    cov_angular = (f)(0.025);
    map_resolution = map_res;
    // Xt = std::make_shared<std::vector<pose>>();
    // Wt = std::make_shared<std::vector<f>>();
    Xt = std::vector<pose>(num_particles);
    Wt = std::vector<f>(num_particles);
    if (seed_x!=0 || seed_y !=0)
    {
        setSeed(seed_x,seed_y);
    }
    init_particles();
    prev_odom.pose.pose.position.x = 0.;
    prev_odom.pose.pose.position.y = 0.;
    prev_odom.pose.pose.position.z = 0.;
    prev_odom.pose.pose.orientation.x = 0.;
    prev_odom.pose.pose.orientation.y = 0.;
    prev_odom.pose.pose.orientation.z = 0.;
    prev_odom.pose.pose.orientation.w = 1.;
    prev_odom.twist.twist.linear.x = 0.;
    prev_odom.twist.twist.linear.y = 0.;
    prev_odom.twist.twist.linear.z = 0.;
    prev_odom.twist.twist.angular.x = 0.;
    prev_odom.twist.twist.angular.y = 0.;
    prev_odom.twist.twist.angular.z = 0.;
    ROS_INFO("Particles initialized");
    bool seed_set;
}

void osm_pf::setSeed(f x, f y)
{
    init_x = x;
    init_y = y;
    seed_set = true;
}

void osm_pf::init_particles()
{
    if(seed_set)
    {
    std::default_random_engine gen;
    std::uniform_real_distribution<f> dist_x(init_x-cov_lin,init_x+cov_lin);
    std::uniform_real_distribution<f> dist_y(init_y-cov_lin,init_y+cov_lin);
    std::uniform_real_distribution<f> dist_theta(0.,(f)2*M_PI);

    for(int i=0;i<num_particles;i++)
    {
        f _x = dist_x(gen);
        f _y = dist_y(gen);
        f _theta = dist_theta(gen);
        pose p(_x,_y,_theta);
        Xt[i]=p;
    } 
    }
    else
    {
    // std::default_random_engine generator;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<f> distribution_x(origin_x,max_x);
    std::uniform_real_distribution<f> distribution_y(origin_y,max_y);
    std::uniform_real_distribution<f> distribution_theta(0,(f)M_PI*2);

    for(int i=0;i<num_particles;i++)
    {
        f _x = distribution_x(gen);
        f _y = distribution_y(gen);
        f _theta = distribution_theta(gen);
        pose p(_x,_y,_theta);
        Xt[i]=p;
    } 

    }
   
}

pose osm_pf::find_xbar(pose x_tminus1,nav_msgs::Odometry odom)
{
    f dx = odom.pose.pose.position.x - prev_odom.pose.pose.position.x;
    f dy = odom.pose.pose.position.y - prev_odom.pose.pose.position.y;
    geometry_msgs::Quaternion q_new = odom.pose.pose.orientation;   
    geometry_msgs::Quaternion q_old = this->prev_odom.pose.pose.orientation;

    f dtheta =  tf::getYaw(q_new) -tf::getYaw(q_old);; //check sign conversions
    this->prev_odom = odom;

    // std::default_random_engine generator;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<f> dist_x(x_tminus1.x+dx,cov_lin);
    std::normal_distribution<f> dist_y(x_tminus1.y+dy,cov_lin);
    std::normal_distribution<f> dist_theta(x_tminus1.theta+dtheta,cov_angular);
    pose p_k_bar(dist_x(gen),dist_y(gen),dist_theta(gen));
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
    auto e = int((point.x - origin_x)/map_resolution);
    auto n = int((point.y - origin_y)/map_resolution);
    auto i = point.intensity;
    f wt,distance,r_w,d2;
    r_w = (f)8;

    if(point.x>origin_x && point.x<max_x && point.y>origin_y && point.y<max_y)
    {
        distance = d_matrix(n,e,0);

    }
    else
    {
        distance = -1.0;
    }
    
    // if(distance<15 && i==255.0)
    // {
    //     std::cout<<"\nDistance less than 15m";
    // }

    if(distance>0.0)
    {
        wt = 1.0 - std::min(distance/r_w,1.0);
        if(i = 255.0)
        {
        //    ROS_INFO("Got road point");
           return wt;
        }
        if(i != 255.0)
        {
            wt = (f)1.0 - wt;
        }

    }
    else
    {
        wt =0.0; //Need to test this
    }
    



    return wt;
    
}

pcl::PointCloud<pcl::PointXYZI>::Ptr osm_pf::drop_zeros(sensor_msgs::PointCloud2 p_cloud)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZI>) ;
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    pcl::fromROSMsg(p_cloud,pcl_cloud);

    // std::cout<<"Incloud size before dropping zeros: "<<pcl_cloud.points.size()<<std::endl;


    for(auto point : pcl_cloud.points)
    {
        if(point.x*point.x > 0.01 && point.y*point.y> 0.01 &&  point.z*point.z> 0.01 & std::sqrt(point.x*point.x+point.y*point.y+point.z*point.z) <10)
        {
            out_cloud->points.push_back(point);
        }
    }
    // std::cout<<"Outcloud size after dropping zeros: "<<out_cloud.points.size()<<std::endl;
    return out_cloud;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr osm_pf::downsize(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr p_cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> filter;
    filter.setInputCloud(incloud);
    filter.setLeafSize(0.2f,0.2f,0.2f);
    filter.filter(*p_cloud_filtered);
    // std::cout<<"\nFiltered!";

    return p_cloud_filtered;
}

osm_pf::f osm_pf::find_wt(pose xbar,sensor_msgs::PointCloud2 p_cloud)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr p_cloud_ptr= drop_zeros(p_cloud);

    pcl::PointCloud<pcl::PointXYZI>::Ptr p_cloud_filtered = downsize(p_cloud_ptr);
    // std::cout<<"Outcloud size in function after  downsampling: "<<p_cloud_filtered->points.size()<<std::endl;
    Eigen::Matrix4f T;
    T<<cos(xbar.theta), -sin(xbar.theta),0, xbar.x,
       sin(xbar.theta), cos(xbar.theta), 0, xbar.y,
       0, 0, 1,0,
       0,0,0,1;
    
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud_map_frame;
    pcl::transformPointCloud(*p_cloud_filtered,pcl_cloud_map_frame,T);
    // std::cout<<"\nCloud Transformed";
    // std::cout<<"\nOutcloud size in function after  transforming: "<<pcl_cloud_map_frame.points.size()<<std::endl;


    
    f weight = 1;
    for(int i=0;i<pcl_cloud_map_frame.points.size();i++)
    {
        pcl::PointXYZI p = pcl_cloud_map_frame.points[i];
        f w = find_wt_point(p);
        // if(w>0.)
        // {
        //     ROS_INFO("valid point");
        // }
        weight = weight+w;
    }
    // std::cout<<"\nWeight calculated";

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
    std::cout<<"\nWeights: "<<std::endl;
    for(auto x : weights)
    {
        std::cout<<" "<<x;
    }
    return weights;
}

std::vector<pose> osm_pf::sample_xt(std::vector<pose> Xbar_t,std::vector<f>& Wt)
{
    std::vector<pose> Xt_gen;
    std::vector<f> Wt_gen;
    // std::default_random_engine generator;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::discrete_distribution<int> distribution(Wt.begin(),Wt.end());

    for(int i=0;i<Xbar_t.size();i++)
    {
        int idx = distribution(gen);
        Xt_gen.push_back(Xbar_t[idx]);
        Wt_gen.push_back(Wt[idx]);
    }
    Wt = Wt_gen;

    return Xt_gen;
}

void osm_pf::callback(const nav_msgs::OdometryConstPtr& u_ptr,const sensor_msgs::PointCloud2ConstPtr& z_ptr)
{
    nav_msgs::Odometry u = *u_ptr;
    sensor_msgs::PointCloud2 z = *z_ptr;
    
    std::vector<pose> Xbar = find_Xbar(Xt,u);
    std::vector<f> Wt_est  = find_Wt(Xbar,z);

    if (count == 5)
    {

        std::vector<pose> X_t_est = sample_xt(Xbar,Wt_est);
        Xt = X_t_est;
        Wt = Wt_est;
        count = 0;

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
        msg.header.frame_id = "map";

        pf_publisher.publish(msg);    
    }

    else
    {
        Xt = Xbar;
        Wt = Wt + Wt_est;

        geometry_msgs::PoseArray msg;
        geometry_msgs::Quaternion q;
        geometry_msgs::Point position;
        geometry_msgs::Pose p;
        for(pose x: Xt)
        {
            q = tf::createQuaternionMsgFromYaw(x.theta);
            position.x = x.x;
            position.y = x.y;
            position.z = 0.0;
            p.position = position;
            p.orientation = q;
            msg.poses.push_back(p);  
        }
        msg.header.frame_id = "map";

        pf_publisher.publish(msg);    
        count++;
    }

}




void osm_pf::run()
{
    sync->registerCallback(boost::bind(&osm_pf::callback,this,_1,_2));
}