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
#include<chrono>
#include<cv_bridge/cv_bridge.h>
#include<numeric>


using namespace osmpf;

osm_pf::f mean(std::vector<osm_pf::f> Wt)
{
    osm_pf::f count = 0;
    for(int i=0;i<Wt.size();i++)
    {
        if (Wt[i]>1e-20)
        {count+=1;}
    }
    return count/Wt.size();
}

void osm_pf::std_dibn()
{
    std::vector<double> x(Xt.size());
    std::vector<double> y(Xt.size());
    double sum_x,sum_y;
    sum_x=0;
    sum_y=0;
     
    for(int i=0;i<Xt.size();i++)
    {
        x[i]=double(Xt[i].x);y[i]=double(Xt[i].y);
        sum_x+=x[i];
        sum_y+=y[i];
    }

    double mean_x = sum_x / x.size();

    std::vector<double> diff_x(x.size());
    std::transform(x.begin(), x.end(), diff_x.begin(), [mean_x](double x) { return x - mean_x; });
    double sq_sum_x = std::inner_product(diff_x.begin(), diff_x.end(), diff_x.begin(), 0.0);
    double stdev_x = std::sqrt(sq_sum_x / x.size());

    double mean_y = sum_y / y.size();

    std::vector<double> diff_y(y.size());
    std::transform(y.begin(), y.end(), diff_y.begin(), [mean_y](double y) { return y - mean_y; });
    double sq_sum_y = std::inner_product(diff_y.begin(), diff_y.end(), diff_y.begin(), 0.0);
    double stdev_y = std::sqrt(sq_sum_y / y.size());

    std::cout<<"Std.dev X: "<<stdev_x<<" Std.dev Y: "<<stdev_y<<std::endl;

    std_x = stdev_x;
    std_y = stdev_y;


}

void osm_pf::update_num_particles()
{
    if (std_x<std_lim && std_y<std_lim)
    {
        // std::cout<<"Grad: "<<m*((std_x+std_y)/2)<<std::endl;
        num_particles = int(m*((std_x+std_y)/2)+min_particles);
        // num_particles = std::min(int(m*((std_x+std_y)/2)),min_particles);
    }

}

template <typename T>
std::vector<T> operator+(const std::vector<T>& a, const std::vector<T>& b)
{
    // assert(a.size() == b.size());

    // std::vector<T> result;
    // result.reserve(a.size());

    // std::transform(a.begin(), a.end(), b.begin(), 
    //                std::back_inserter(result), std::plus<T>());
    // return result;
    std::vector<T> result;
    for(int i=0;i<a.size();i++)
    {
        result.push_back(a[i]+b[i]);
    }
    return result;
}

template <typename T>
std::vector<T> operator*(const std::vector<T>& a, const std::vector<T>& b)
{
    // assert(a.size() == b.size());

    // std::vector<T> result;
    // result.reserve(a.size());

    // std::transform(a.begin(), a.end(), b.begin(), 
    //                std::back_inserter(result), std::plus<T>());
    // return result;
    std::vector<T> result;
    for(int i=0;i<a.size();i++)
    {
        result.push_back(a[i]*b[i]);
    }
    return result;
}

template <typename T>
std::vector<T> multiply_sum_sq(const std::vector<T>& a, const std::vector<T>& b,osm_pf::f& w_sum_sq)
{
    // assert(a.size() == b.size());

    // std::vector<T> result;
    // result.reserve(a.size());

    // std::transform(a.begin(), a.end(), b.begin(), 
    //                std::back_inserter(result), std::plus<T>());
    // return result;
    std::vector<T> result;
    for(int i=0;i<a.size();i++)
    {
        result.push_back(a[i]*b[i]);
        // w_sum_sq+=(a[i]*b[i])*(a[i]*b[i]);
        w_sum_sq+=(b[i]*b[i]);
    }
    return result;
}

template <typename T>
std::vector<T> sum_sum_sq(const std::vector<T>& a, const std::vector<T>& b,osm_pf::f& w_sum_sq)
{
    // assert(a.size() == b.size());

    // std::vector<T> result;
    // result.reserve(a.size());

    // std::transform(a.begin(), a.end(), b.begin(), 
    //                std::back_inserter(result), std::plus<T>());
    // return result;
    std::vector<T> result;
    for(int i=0;i<a.size();i++)
    {
        result.push_back(a[i]+b[i]);
        w_sum_sq+=(a[i]+b[i])*(a[i]+b[i]);
    }
    return result;
}


osm_pf::osm_pf(std::string path_to_d_mat,f min_x,f min_y,f Max_x,f Max_y,f map_res_x,f map_res_y,int particles,f seed_x,f seed_y,bool mono)
{
    num_particles = particles;
    max_particles = particles;   
    mono_mode = mono;
    std_x = 10000.0;
    std_y = 10000.0;

    d_matrix = xt::load_npy<f>(path_to_d_mat);
    count = 0;
    origin_x = min_x;
    origin_y = min_y;
    max_x = Max_x;
    max_y = Max_y;
    map_resolution_x  =map_res_x;
    map_resolution_y  =map_res_y;
    
    nh.getParam("/osm_particle_filter/down_sample_size",down_sample_size);
    nh.getParam("/osm_particle_filter/init_cov_linear",init_cov_linear);
    nh.getParam("/osm_particle_filter/init_cov_angular",init_cov_angular);
    nh.getParam("/osm_particle_filter/odom_cov_lin",odom_cov_lin);
    nh.getParam("/osm_particle_filter/odom_cov_angular",odom_cov_angular);
    nh.getParam("/osm_particle_filter/resampling_count",resampling_count);
    nh.getParam("/osm_particle_filter/use_pi_weighting",use_pi_weighting);
    nh.getParam("/osm_particle_filter/use_pi_resampling",use_pi_resampling);
    nh.getParam("/osm_particle_filter/road_width",road_width);
    nh.getParam("/osm_particle_filter/pi_gain",pi_gain);
    nh.getParam("/osm_particle_filter/queue_size",queue_size);
    nh.getParam("/osm_particle_filter/sync_queue_size",sync_queue_size);
    nh.getParam("/osm_particle_filter/project_cloud",project_cloud);
    nh.getParam("/osm_particle_filter/use_dynamic_resampling",use_dynamic_resampling);
    nh.getParam("/osm_particle_filter/estimate_gps_error",estimate_gps_error);
    nh.getParam("/osm_particle_filter/weight_function",weight_function);
    nh.getParam("/osm_particle_filter/min_particles",min_particles);
    nh.getParam("/osm_particle_filter/std_lim",std_lim);
    nh.getParam("/osm_particle_filter/adaptive_mode",adaptive_mode);
    
    m = (max_particles-min_particles)/std_lim;

    // Initialize Random generators
    gen.reset(new std::default_random_engine);
    dist_ptr.reset(new std::normal_distribution<f>(osm_pf::f(0.0),odom_cov_lin));
    dist_x = *dist_ptr;
    dist_ptr.reset(new std::normal_distribution<f>(osm_pf::f(0.0),odom_cov_angular));
    dist_theta = *dist_ptr;
    
    // dist_theta.reset(new std::normal_distribution<f>(osm_pf::f(0.0),odom_cov_angular));
    // dist_x.reset(osm_pf::f(0.0),odom_cov_lin);
    // Initialize ROS attributes
    pf_publisher = nh.advertise<geometry_msgs::PoseArray>("osm_pose_estimate",100);
    pf_lat_lon = nh.advertise<geometry_msgs::PoseArray>("osm_lat_lon",100);
    pf_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("pf_cloud_map_frame",100);
    pf_avg_pub = nh.advertise<geometry_msgs::PoseStamped>("osm_average_pose_estimate",100);
    // pf_pose = nh.advertise<geometry_msgs::Pose>("osm_weighted_pose",100);
    odom_sub.subscribe(nh,"/vehicle/odometry",queue_size);

    pc_sub.subscribe(nh,"/road_points",queue_size);
    // pc_sub.subscribe(nh,"/road_points",100);
    sync.reset(new Sync(sync_policy(sync_queue_size),odom_sub,pc_sub)) ;
    



    // Xt = std::make_shared<std::vector<pose>>();
    // Wt = std::make_shared<std::vector<f>>();
    Xt = std::vector<pose>(num_particles);
    w_sum_sq = 1/(3*static_cast<f>(num_particles));
    if(use_pi_weighting && use_pi_resampling)
    {
        Wt = std::vector<f>(num_particles,1.0);
    }
    else if (use_pi_weighting && !use_pi_resampling)
    {
        Wt  = std::vector<f>(num_particles,1.0);
    }
    else if(!use_pi_weighting && use_pi_resampling)
    
    {
        Wt  = std::vector<f>(num_particles,1.0);
    }
    else
    {
        Wt  = std::vector<f>(num_particles,0.0);
    }
    bool seed_set=false;
    if (seed_x!=0.0 || seed_y !=0.0)
    {
        setSeed(seed_x,seed_y);
    }
    init_particles();
    prev_odom.pose.pose.position.x = 0.;
    prev_odom.pose.pose.position.y = 0.;
    prev_odom.pose.pose.position.z = 0.;
    // prev_odom.pose.pose.orientation.x = 0.;
    // prev_odom.pose.pose.orientation.y = 0.;
    // prev_odom.pose.pose.orientation.z = 0.;
    // prev_odom.pose.pose.orientation.w = 1.;
    prev_odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    prev_odom.twist.twist.linear.x = 0.;
    prev_odom.twist.twist.linear.y = 0.;
    prev_odom.twist.twist.linear.z = 0.;
    prev_odom.twist.twist.angular.x = 0.;
    prev_odom.twist.twist.angular.y = 0.;
    prev_odom.twist.twist.angular.z = 0.;
    std::cout<<"\n Weight sum sq initialized to: "<< w_sum_sq;
    std::cout<<"\n Neff initialized to: "<< 1/w_sum_sq;
    ROS_INFO("\nParticles initialized");

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
    std::uniform_real_distribution<f> dist_x(init_x-init_cov_linear,init_x+init_cov_linear);
    std::uniform_real_distribution<f> dist_y(init_y-init_cov_linear,init_y+init_cov_linear);
    std::uniform_real_distribution<f> dist_theta(0.,(f)2*init_cov_angular);

    for(int i=0;i<num_particles;i++)
    {
        f _x = dist_x(gen);
        f _y = dist_y(gen);
        f _theta = dist_theta(gen);
        pose p(_x,_y,_theta);
        Xt[i]=p;
    } 
    }
    else // no seed so generate particles all over the map
    {
    // std::default_random_engine generator;
    std::cout<< "\nNo seed given. Initializing Global Localization"<<std::endl;
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

pose osm_pf::find_xbar(pose x_tminus1,f dx,f dy, f dtheta)
{

    // f alpha = x_tminus1.theta + atan2(dy,dx);
    // f e = x_tminus1.x + dr*cos(alpha);
    // f n = x_tminus1.y + dr*sin(alpha);
    f e = x_tminus1.x + (dx*cos(x_tminus1.theta))-(dy*sin(x_tminus1.theta ));
    f n = x_tminus1.y +(dx*sin(x_tminus1.theta))+ (dy*cos(x_tminus1.theta));
    f theta =  x_tminus1.theta+dtheta;
    // std::cout<<"\n ***** Odometry Debug*** ";
    // std::cout<<"\n x_tminus1.x = "<<x_tminus1.x<<" x_tminus1.y = "<<x_tminus1.x; 
    // std::cout<<"\n dx = "<<dx<<" dy = "<<dy; 
    // std::cout<<"\n x_t.x = "<<e<<" x_t.y = "<<n;
    // std::cout<<"\n theta_prev: "<<tf::getYaw(q_new)<<" theta_new"<<tf::getYaw(q_old)<<" dtheta:"<<dtheta;
    // std::cout<<"\n x_tmins1.theta = "<<x_tminus1.theta<<" dtheta:"<<dtheta<<" x_t.theta = "<<theta;  
    // alpha = alpha +dtheta;
    // std::default_random_engine generator;


    pose p_k_bar(e+dist_x(*gen),n+dist_x(*gen),theta+dist_theta(*gen));
    return p_k_bar;
}

std::vector<pose> osm_pf::find_Xbar(std::vector<pose> X_tminus1,nav_msgs::Odometry odom)
{
    // Find change in pose (same for all particles)
    f delta_x_odom_frame = odom.pose.pose.position.x - prev_odom.pose.pose.position.x;
    f delta_y_odom_frame = odom.pose.pose.position.y - prev_odom.pose.pose.position.y;    
    geometry_msgs::Quaternion q_new = odom.pose.pose.orientation;   
    geometry_msgs::Quaternion q_old = prev_odom.pose.pose.orientation;

    f dtheta =  tf::getYaw(q_new) -tf::getYaw(q_old);; //check sign conversions
    f delta_r = sqrt((delta_x_odom_frame*delta_x_odom_frame)+(delta_y_odom_frame*delta_y_odom_frame));
    f alpha = atan2(delta_y_odom_frame,delta_x_odom_frame)-tf::getYaw(q_old);
    
    f dx = delta_r*cos(alpha);
    f dy = delta_r*sin(alpha);


    std::vector<pose> Xt_bar(num_particles);
    for(int i=0;i<num_particles;i++)
    {
        Xt_bar[i] = find_xbar(X_tminus1[i],dx,dy,dtheta);
    }
    prev_odom = odom;
    return Xt_bar;
    

}

osm_pf::f osm_pf::weightfunction(f distance,f road_width,f intensity)
{
    if(weight_function=="gaussian")
    {

        return (1/(road_width*sqrt(2*3.14)))*exp(-1*((std::pow(distance,2))/(2*std::pow(road_width,2))));

    }
    else if(weight_function=="maplite")
    {
        return 1.0 - std::min(distance/road_width,1.0);;
    }
    else if (weight_function=="quadratic")
    {   
        distance = static_cast<f>(abs(distance));
        if(distance<road_width && intensity>127.0)
        {
            return static_cast<f>(1.0);    
        }
        // distance = distance/road_width;
        // f x = std::min(distance/road_width,1.0);
        // if (x==1.0)
        // {
        //     x = 10.0;
        // }
        return 1.0/(std::pow(distance,2)+1.0);
    }
    else if(weight_function=="exp_decay")
    {
        distance = static_cast<f>(abs(distance));
        if(distance<road_width && intensity>127.0)
        {
            return static_cast<f>(1.0);    
        }
        else
        {
            f w  = 1/(exp((distance/road_width)*1.2));
            // std::cout<<"\nWeight point:"<<w;
            return w;

        }


    }
    else
    {
        ROS_ERROR(" No match for weighting function provided. Values must be one of: gaussian,maplite,exp_decay or quadratic");
        // throw ros::InvalidParameterException p();

    }
    
}

osm_pf::f osm_pf::find_wt_point(pcl::PointXYZI point)
{
    int e = (int((std::round((point.x - origin_x)/map_resolution_x))) )-1;
    int n = (int(std::round((point.y - origin_y)/map_resolution_y)))-1;
    int intensity = int(point.intensity);
    f wt,distance,r_w,d2;
    r_w = (f)road_width;
    // std::cout<<"\n e:"<<e<<" n: "<<n<<" intensity:"<<intensity;

    // std::cout<<"\nShape n:"<<d_matrix.shape()[1]<<" e:"<<d_matrix.shape()[1]<<std::endl;

    // if(point.x>origin_x && point.x<max_x && point.y>origin_y && point.y<max_y)
    // {
    //     distance = d_matrix(n,e,0);

    // }
    if(e>=0 && e< d_matrix.shape()[1] && n>=0  && n<d_matrix.shape()[0])
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
    // std::cout<<"\n Distance:"<<distance<<std::endl;
    if(distance>0.0)
    {
        wt = weightfunction(distance,r_w,intensity);
        if(intensity > 127)
        {
            // std::cout<<"\n Point is a road point, intensity="<<intensity<<std::endl;
            if(use_pi_weighting)
            {
                return wt;
            }            

            else
            {
                if (wt>0.7)
                    {   
                        // ROS_INFO("Got road point");
                        return 1000.0;
                    }
                if (wt>0.5)
                {   
                    // ROS_INFO("Got road point");
                    return 5.0;
                }

                if (wt>0.1 && wt<0.5)
                {   
                    // ROS_INFO("Got road point");
                    return 0.5;
                }
                else
                {
                    return -1.0;
                }
            }
            

        }
           
        
        else
        {
            wt= static_cast<f>(1.0) - wt;
            if(use_pi_weighting)
            {
                return wt;
            }


            else
            {
                    if (wt>0.7)
                {   
                    // ROS_INFO("Got road point");
                    return 1000.0;
                }
                if (wt>0.5)
                {   
                    // ROS_INFO("Got road point");
                    return 5.0;
                }

                if (wt>0.1 && wt<0.5)
                {   
                    // ROS_INFO("Got road point");
                    return 0.5;
                }
                else
                {
                    return -1.0;
                }
            }


        }

    }
    else
    {
        // wt =-10.0; //Need to test this
        if(use_pi_weighting)
        {
            return 0.1;
        }
        else
        {
            return 0.0;
        }
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
    filter.setLeafSize(down_sample_size,down_sample_size,down_sample_size);
    filter.filter(*p_cloud_filtered);
    // std::cout<<"\nFiltered!";

    return p_cloud_filtered;
}

osm_pf::f osm_pf::find_wt(pose xbar,pcl::PointCloud<pcl::PointXYZI>::Ptr p_cloud_filtered)
{

    Eigen::Matrix4f T;
    T<<cos(xbar.theta), -sin(xbar.theta),0, xbar.x,
       sin(xbar.theta), cos(xbar.theta), 0, xbar.y,
       0, 0, 1,0,
       0,0,0,1;
    
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud_map_frame;

    pcl::transformPointCloud(*p_cloud_filtered,pcl_cloud_map_frame,T);

    // std::cout<<"\nCloud Transformed";
    // std::cout<<"\nOutcloud size in function after  transforming: "<<pcl_cloud_map_frame.points.size()<<std::endl;

    f weight;
    f w;
    if (use_pi_weighting)
    {
        weight = 1.0;    
    }
    else
    {
        weight = 0.0;
    }
    
    for(int i=0;i<pcl_cloud_map_frame.points.size();i++)
    {
        pcl::PointXYZI p = pcl_cloud_map_frame.points[i];
        w = find_wt_point(p);
        // std::cout<<"\nGot weight="<<w;
        if(use_pi_weighting)
        {
            weight = weight*w*pi_gain;
        }
        else
        {
            weight = weight+w;
        }

        
    }

    // std::cout<<"\nWeight calculated at curent step: "<<weight<<std::endl;

    return weight;


}

std::vector<osm_pf::f> osm_pf::find_Wt(std::vector<pose> Xtbar,sensor_msgs::PointCloud2 p_cloud)
{
    // Preprocess PointCloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr p_cloud_ptr;
    if(!mono_mode)
    {
        p_cloud_ptr= drop_zeros(p_cloud);
    }
    else
    {
        p_cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(p_cloud,*p_cloud_ptr);

    }


    p_cloud_filtered = downsize(p_cloud_ptr);
    std::vector<f> weights;
    f weight;
    for(pose p: Xtbar)
    {
        weight=find_wt(p,p_cloud_filtered);
        weights.push_back(weight);
    }
    return weights;
}

std::vector<pose> osm_pf::sample_xt(std::vector<pose> Xbar_t,std::vector<f>& Wt)
{
    std::vector<pose> Xt_gen(num_particles);
    // std::vector<f> Wt_gen;
    // std::default_random_engine generator;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::discrete_distribution<int> distribution(Wt.begin(),Wt.end());

    for(int i=0;i<num_particles;i++)
    {
        int idx = distribution(gen);
        Xt_gen[i]= Xbar_t[idx];
        // Wt_gen.push_back(Wt[idx]);
    }
    // Wt = Wt_gen;
    Wt = std::vector<f>(num_particles,1.0);

    return Xt_gen;
}

std::shared_ptr<pose> osm_pf::weight_pose(std::vector<pose> Poses,std::vector<f> Weights)
{
    f avg_x,avg_y,avg_theta,weight_sum,wt_norm = 0.0 ;
    std::shared_ptr<pose> avg_pose(new pose);
    f max_wt = *std::max_element(Weights.begin(),Weights.end());

    for(int i=0;i<Poses.size();i++)
    {
        wt_norm = Weights[i]/max_wt;
        avg_x += wt_norm*Poses[i].x;
        avg_y+= wt_norm*Poses[i].y;
        avg_theta+= wt_norm*Poses[i].theta;
        weight_sum+= wt_norm;
    }

    avg_x = avg_x/weight_sum;
    avg_y = avg_y/weight_sum;
    avg_theta = avg_theta/weight_sum;

    avg_pose->x = avg_x;
    avg_pose->y = avg_y;
    avg_pose->theta = avg_theta;

    return avg_pose;

}

void osm_pf::publish_msg(std::vector<pose> X,std::vector<f> W,std_msgs::Header h)
{
    geometry_msgs::PoseArray msg;
    geometry_msgs::PoseArray msg_lat_lon;
    geometry_msgs::Quaternion q;
    geometry_msgs::Point position;
    geometry_msgs::Pose p;
    geometry_msgs::PoseStamped pose_avg;
    tf::Transform osm_tf;
    tf::Quaternion tf_q;

    f avg_x=0.0;
    f avg_y=0.0;
    f avg_theta=0.0;
    f weight_sum=0.0;
    f wt_norm = 0.0 ;
    std::vector<f>::iterator max_wt_it = std::max_element(W.begin(),W.end());
    f max_wt = *max_wt_it;
    int max_idx = std::distance(W.begin(),max_wt_it);

    float lat,lon;
    for(int i=0;i<X.size();i++)
    {
        q = tf::createQuaternionMsgFromYaw(X[i].theta);
        position.x = X[i].x;
        position.y = X[i].y;
        position.z = 0.0;
        p.position = position;
        p.orientation = q;
        msg.poses.push_back(p); 

        // wt_norm = (W[i]+0.00000001)/max_wt;
        wt_norm = W[i];
        avg_x += wt_norm*X[i].x;
        avg_y+= wt_norm*X[i].y;
        avg_theta+= wt_norm*X[i].theta;
        weight_sum+= wt_norm;


        
        // UTMXYToLatLon(X[i].x,X[i].y,14,false,lat,lon);
        // position.x = lat;
        // position.y = lon;
        // position.z = 0.0;
        // p.position = position;
        // p.orientation = q;
        // msg_lat_lon.poses.push_back(p); 
    }
    avg_x = avg_x/weight_sum;
    avg_y = avg_y/weight_sum;
    avg_theta = avg_theta/weight_sum;

    if(isnan(avg_x) || isnan(avg_y) || isnan(avg_theta))
    {
        // pose_avg.pose.orientation = tf::createQuaternionMsgFromYaw(X[max_idx].theta);
        // pose_avg.pose.position.x = X[max_idx].x;
        // pose_avg.pose.position.y = X[max_idx].y;
        // pose_avg.header.stamp = h.stamp;
        // pose_avg.header.frame_id = "map";

        // pf_avg_pub.publish(pose_avg);

    }

    else
    {
        pose_avg.pose.orientation = tf::createQuaternionMsgFromYaw(avg_theta);
        pose_avg.pose.position.x = avg_x;
        pose_avg.pose.position.y = avg_y;
        pose_avg.header.stamp = h.stamp;
        pose_avg.header.frame_id = "map";



        pf_avg_pub.publish(pose_avg);

       if(project_cloud)
        {

;
        sensor_msgs::PointCloud2 pc_cloud_msg;
        pcl::toROSMsg(*p_cloud_filtered,pc_cloud_msg);
        pc_cloud_msg.header.frame_id  = "osm_pose_estimate";
        pc_cloud_msg.header.stamp = h.stamp;
        pf_cloud_pub.publish(pc_cloud_msg);
        }


    }
    // std::shared_ptr<pose> avg_pose = weight_pose(X,W);
    osm_tf.setOrigin(tf::Vector3(avg_x,avg_y,0.0));
    tf_q.setRPY(0.0,0.0,avg_theta);
    osm_tf.setRotation(tf_q);
    osm_pose_broadcaster.sendTransform(tf::StampedTransform(osm_tf,ros::Time::now(),"map","osm_pose_estimate"));


    msg.header.frame_id = "map";
    // msg_lat_lon.header.frame_id = "map";

    pf_publisher.publish(msg);    
    // pf_lat_lon.publish(msg_lat_lon);  

}


void osm_pf_stereo::publish_msg_stereo(std::vector<pose> X,std::vector<f> W,std_msgs::Header h,const sensor_msgs::PointCloud2& ip_cloud)
{
    geometry_msgs::PoseArray msg;
    geometry_msgs::PoseArray msg_lat_lon;
    geometry_msgs::Quaternion q;
    geometry_msgs::Point position;
    geometry_msgs::Pose p;
    geometry_msgs::PoseStamped pose_avg;
    static tf::TransformBroadcaster br;
    tf::Transform osm_tf;
    tf::Quaternion tf_q;

    f avg_x,avg_y,avg_theta,weight_sum,wt_norm = 0.0 ;
    std::vector<f>::iterator max_wt_it = std::max_element(W.begin(),W.end());
    f max_wt = *max_wt_it+0.00000001;
    int max_idx = std::distance(W.begin(),max_wt_it);

    float lat,lon;
    for(int i=0;i<X.size();i++)
    {
        q = tf::createQuaternionMsgFromYaw(X[i].theta);
        position.x = X[i].x;
        position.y = X[i].y;
        position.z = 0.0;
        p.position = position;
        p.orientation = q;
        msg.poses.push_back(p); 

        wt_norm = (W[i]+0.00000001)/max_wt;
        avg_x += wt_norm*X[i].x;
        avg_y+= wt_norm*X[i].y;
        avg_theta+= wt_norm*X[i].theta;
        weight_sum+= wt_norm;


        
        // UTMXYToLatLon(X[i].x,X[i].y,14,false,lat,lon);
        // position.x = lat;
        // position.y = lon;
        // position.z = 0.0;
        // p.position = position;
        // p.orientation = q;
        // msg_lat_lon.poses.push_back(p); 
    }
    avg_x = avg_x/weight_sum;
    avg_y = avg_y/weight_sum;
    avg_theta = avg_theta/weight_sum;

    if(isnan(avg_x) || isnan(avg_y) || isnan(avg_theta))
    {
        // pose_avg.pose.orientation = tf::createQuaternionMsgFromYaw(X[max_idx].theta);
        // pose_avg.pose.position.x = X[max_idx].x;
        // pose_avg.pose.position.y = X[max_idx].y;
        // pose_avg.header.stamp = h.stamp;
        // pose_avg.header.frame_id = "map";

        // pf_avg_pub.publish(pose_avg);

    }

    else
    {
        pose_avg.pose.orientation = tf::createQuaternionMsgFromYaw(avg_theta);
        pose_avg.pose.position.x = avg_x;
        pose_avg.pose.position.y = avg_y;
        pose_avg.header.stamp = h.stamp;
        pose_avg.header.frame_id = "map";


        pf_avg_pub.publish(pose_avg);


    //    if(project_cloud)
    //     {
    //         pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud;
    //         pcl::fromROSMsg(ip_cloud,*pcl_cloud);

    //         Eigen::Matrix4f T;
    //         T<<cos(avg_theta), -sin(avg_theta),0, avg_x,
    //         sin(avg_theta), cos(avg_theta), 0, avg_x,
    //         0, 0, 1,0,
    //         0,0,0,1;
            
    //         pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud_map_frame;

    //         pcl::transformPointCloud(*pcl_cloud,pcl_cloud_map_frame,T);
    //         sensor_msgs::PointCloud2 pc_cloud_msg;
    //         pcl::toROSMsg(pcl_cloud_map_frame,pc_cloud_msg);
    //         pc_cloud_msg.header.frame_id  = "map";
    //         pf_cloud_pub.publish(pc_cloud_msg);
    //     }
    }
    // std::shared_ptr<pose> avg_pose = weight_pose(X,W);

    osm_tf.setOrigin(tf::Vector3(avg_x,avg_y,0.0));
    tf_q.setRPY(0.0,0.0,avg_theta);
    osm_tf.setRotation(tf_q);
    br.sendTransform(tf::StampedTransform(osm_tf,ros::Time::now(),"map","osm_pose_estimate"));


    msg.header.frame_id = "map";
    // msg_lat_lon.header.frame_id = "map";

    pf_publisher.publish(msg);    
    // pf_lat_lon.publish(msg_lat_lon);  

}

void osm_pf::callback(const nav_msgs::OdometryConstPtr& u_ptr,const sensor_msgs::PointCloud2ConstPtr& z_ptr)
{
    nav_msgs::Odometry u = *u_ptr;
    sensor_msgs::PointCloud2 z = *z_ptr;
    
    std::vector<pose> Xbar = find_Xbar(Xt,u);

    std::vector<f> Wt_est  = find_Wt(Xbar,z);
    // std::cout<<"\n Initial Weights calculated \n";

    if(use_dynamic_resampling)
    {
        f n_eff = (1.0/w_sum_sq);
        // std::cout<<"\n Weight sum sq: "<< w_sum_sq;
        // std::cout<<"\n Number of effective particles: "<< n_eff;
        if(n_eff < (((num_particles))/10) || count>resampling_count)
        {
            std_dibn();
            // std::cout<<"Number of particles: "<<num_particles<<std::endl;
            if(adaptive_mode)
            {
                // std::cout<<"Updateing num particles..\n";
                update_num_particles();
            }
            // std::cout<<"\n -------------- Sampling Weights----- "<<std::endl;
            std::vector<pose> X_t_est = sample_xt(Xbar,Wt_est);
            Xt = X_t_est;
            Wt = Wt_est;
            // std::cout<<"\nWeights: "<<std::endl;
            // for(auto x : Wt)
            // {
            //     std::cout<<" "<<x;
            // }
            count = 0;

            publish_msg(X_t_est,Wt_est,u.header);
            w_sum_sq = 1/(3*static_cast<f>(num_particles));
            // std::cout<<"\n Weight sum sq: "<< w_sum_sq;

        }
        else
        {
            // std::cout<<"\n -------------- Not Sampling Weights----- "<<std::endl;
            Xt = Xbar;

            if (use_pi_resampling)
            {
                Wt = Wt * Wt_est;
            }
            else
            {
            Wt = Wt + Wt_est;
            }

            w_sum_sq = std::inner_product(Wt.begin(),Wt.end(),Wt.begin(),0);

            
            // if(w_sum_sq == 0.0)
            // {
            //     std::cout<<"\n Resetting Weights due to convergence to zero "<<std::endl;
            //     w_sum_sq = 1/num_particles;
            //     Wt  = std::vector<f>(num_particles,1.0);

            // }


            // std::cout<<"\nWeights: "<<std::endl;
            // for(auto x : Wt)
            // {
            //     std::cout<<" "<<x;
            // }


            publish_msg(Xt,Wt,u.header);
            count++;

        }


    }
    else
    {
        if (count == resampling_count)
        {
            std_dibn();
            // std::cout<<"Number of particles: "<<num_particles<<std::endl;
            if(adaptive_mode)
            {
                // std::cout<<"Updateing num particles..\n";
                update_num_particles();
            }
            std::vector<pose> X_t_est = sample_xt(Xbar,Wt_est);
            Xt = X_t_est;
            Wt = Wt_est;

            // std::cout<<"\nWeights: "<<std::endl;
            // for(auto x : Wt)
            // {
            //     std::cout<<" "<<x;
            // }
            count = 0;

            publish_msg(Xt,Wt,u.header);
            
            
        }

        else
        {
            // avg_wt = mean(Wt_est);
            // std::cout<<"Average percentage:"<<avg_wt<<std::endl;
            Xt = Xbar;
            if (use_pi_resampling)
            {
                Wt = Wt * Wt_est;
            }
            else
            {
            Wt = Wt + Wt_est;
            }


            // std::cout<<"\nWeights: "<<std::endl;
            // for(auto x : Wt)
            // {
            //     std::cout<<" "<<x;
            // }
            publish_msg(Xt,Wt,u.header);
            count++;

            
        }
    }

}


void osm_pf::run()
{
    sync->registerCallback(boost::bind(&osm_pf::callback,this,_1,_2));
}



osm_pf_stereo::osm_pf_stereo(std::string path_to_d_mat,f min_x,f min_y,f Max_x,f Max_y,f map_res_x,f map_res_y,int particles,f seed_x,f seed_y,bool mono):  osm_pf{path_to_d_mat,min_x,min_y,Max_x,Max_y,map_res_x,map_res_y,particles,seed_x,seed_y,mono}
{
}

void osm_pf_stereo::callback_s(const nav_msgs::OdometryConstPtr& u_ptr,const sensor_msgs::PointCloud2ConstPtr& z_ptr)
{
    nav_msgs::Odometry u = *u_ptr;
    sensor_msgs::PointCloud2 z = *z_ptr;
    
    // auto start = std::chrono::high_resolution_clock::now();
    std::vector<pose> Xbar = find_Xbar(Xt,u);
    // auto stop = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start);
    // std::cout<<"Translation took: "<<duration.count()<<std::endl;

    // start = std::chrono::high_resolution_clock::now();
    std::vector<f> Wt_est  = find_Wt_s(Xbar,z);
    // stop = std::chrono::high_resolution_clock::now();
    // duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start);
    // std::cout<<"Weight updating took: "<<duration.count()<<std::endl;
    // std::cout<<"\n Initial Weights calculated \n";

    if(use_dynamic_resampling)
    {
        f n_eff = (1/w_sum_sq);
        // std::cout<<"\n Weight sum sq: "<< w_sum_sq;
        // std::cout<<"\n Number of effective particles: "<< n_eff;
        if(n_eff < ((num_particles)/10) || count>20)
        {
            // std::cout<<"\n -------------- Sampling Weights----- "<<std::endl;
            std::vector<pose> X_t_est = sample_xt(Xbar,Wt_est);
            Xt = X_t_est;
            Wt = Wt_est;
            // std::cout<<"\nWeights: "<<std::endl;
            // for(auto x : Wt)
            // {
            //     std::cout<<" "<<x;
            // }
            count = 0;

            publish_msg_stereo(X_t_est,Wt_est,u.header,z);
            w_sum_sq = 1/(3*static_cast<f>(num_particles));
            // std::cout<<"\n Weight sum sq: "<< w_sum_sq;

        }
        else
        {
            // std::cout<<"\n -------------- Not Sampling Weights----- "<<std::endl;
            Xt = Xbar;
            w_sum_sq = 0.0;

            
            Wt = multiply_sum_sq(Wt, Wt_est,w_sum_sq);
            // std::cout<<"\n Weight sum sq: "<< w_sum_sq;
            count += 0;

            
            // if(w_sum_sq == 0.0)
            // {
            //     std::cout<<"\n Resetting Weights due to convergence to zero "<<std::endl;
            //     w_sum_sq = 1/num_particles;
            //     Wt  = std::vector<f>(num_particles,1.0);

            // }


            // std::cout<<"\nWeights: "<<std::endl;
            // for(auto x : Wt)
            // {
            //     std::cout<<" "<<x;
            // }


            publish_msg_stereo(Xt,Wt,u.header,z);
            count++;

        }


    }
    else
    {
        if (count == resampling_count)
        {
            std_dibn();
            // std::cout<<"Number of particles: "<<num_particles<<std::endl;
            if(adaptive_mode)
            {
                update_num_particles();
            }
            std::vector<pose> X_t_est = sample_xt(Xbar,Wt_est);
            Xt = X_t_est;
            Wt = Wt_est;

            // std::cout<<"\nWeights: "<<std::endl;
            // for(auto x : Wt)
            // {
            //     std::cout<<" "<<x;
            // }
            count = 0;

            publish_msg_stereo(Xt,Wt,u.header,z);
            
            
        }

        else
        {
            // avg_wt = mean(Wt_est);
            // std::cout<<"Average percentage:"<<avg_wt<<std::endl;
            Xt = Xbar;
            if (use_pi_resampling)
            {
                Wt = Wt * Wt_est;
            }
            else
            {
            Wt = Wt + Wt_est;
            }


            // std::cout<<"\nWeights: "<<std::endl;
            // for(auto x : Wt)
            // {
            //     std::cout<<" "<<x;
            // }
            publish_msg_stereo(Xt,Wt,u.header,z);
            count++;
            // std_dibn();
            // std::cout<<"Number of particles: "<<num_particles<<std::endl;
            
        }
    }

}

void osm_pf_stereo::run_s()
{
    sync->registerCallback(boost::bind(&osm_pf_stereo::callback_s,this,_1,_2));
}

osm_pf_stereo::f osm_pf_stereo::find_wt_point_s(pcl::PointXYZRGB point)
{
    int e = (int((std::round((point.x - origin_x)/map_resolution_x))) )-1;
    int n = (int(std::round((point.y - origin_y)/map_resolution_y)))-1;
    int intensity = int(point.rgb);
    f wt,distance,r_w,d2;
    // r_w = (f)road_width;
    // std::cout<<"\n e:"<<e<<" n: "<<n<<" intensity:"<<intensity;

    // std::cout<<"\nShape n:"<<d_matrix.shape()[1]<<" e:"<<d_matrix.shape()[1]<<std::endl;

    // if(point.x>origin_x && point.x<max_x && point.y>origin_y && point.y<max_y)
    // {
    //     distance = d_matrix(n,e,0);

    // }
    if(e>=0 && e< d_matrix.shape()[1] && n>=0  && n<d_matrix.shape()[0])
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
    // std::cout<<"\n Distance:"<<distance<<std::endl;
    if(distance>0.0)
    {
        wt = weightfunction(distance,f(road_width),intensity);
        if(intensity > 127)
        {
            // std::cout<<"\n Point is a road point, intensity="<<intensity<<std::endl;
            if(use_pi_weighting)
            {
                return wt;
            }            

            else
            {
                if (wt>0.7)
                    {   
                        // ROS_INFO("Got road point");
                        return 1000.0;
                    }
                if (wt>0.5)
                {   
                    // ROS_INFO("Got road point");
                    return 5.0;
                }

                if (wt>0.1 && wt<0.5)
                {   
                    // ROS_INFO("Got road point");
                    return 0.5;
                }
                else
                {
                    return -1.0;
                }
            }
            

        }
           
        
        else
        {
            wt= static_cast<f>(1.0) - wt;
            if(use_pi_weighting)
            {
                return wt;
            }


            else
            {
                    if (wt>0.7)
                {   
                    // ROS_INFO("Got road point");
                    return 1000.0;
                }
                if (wt>0.5)
                {   
                    // ROS_INFO("Got road point");
                    return 5.0;
                }

                if (wt>0.1 && wt<0.5)
                {   
                    // ROS_INFO("Got road point");
                    return 0.5;
                }
                else
                {
                    return -1.0;
                }
            }


        }

    }
    else
    {
        // wt =-10.0; //Need to test this
        if(use_pi_weighting)
        {
            return 0.1;
        }
        else
        {
            return 0.0;
        }
    }
    



    return wt;
    
}

osm_pf_stereo::f osm_pf_stereo::find_wt_s(pose xbar,pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud_filtered)
{

    Eigen::Matrix4f T;
    T<<cos(xbar.theta), -sin(xbar.theta),0, xbar.x,
       sin(xbar.theta), cos(xbar.theta), 0, xbar.y,
       0, 0, 1,0,
       0,0,0,1;
    
    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud_map_frame;

    pcl::transformPointCloud(*p_cloud_filtered,pcl_cloud_map_frame,T);


    // std::cout<<"\nCloud Transformed";
    // std::cout<<"\nOutcloud size in function after  transforming: "<<pcl_cloud_map_frame.points.size()<<std::endl;

    osm_pf_stereo::f weight;
    osm_pf_stereo::f w;
    if (use_pi_weighting)
    {
        weight = 1.0;    
    }
    else
    {
        weight = 0.0;
    }
 

    for(int i=0;i<pcl_cloud_map_frame.points.size();i++)
    {
        
        pcl::PointXYZRGB p = pcl_cloud_map_frame.points[i];
        w = find_wt_point_s(p);
        // std::cout<<"\nGot weight="<<w;
        if(use_pi_weighting)
        {
            weight = weight*w*pi_gain;
        }
        else
        {
            weight = weight+w;
        }

        
    }


    // std::cout<<"\nWeight calculated at curent step: "<<weight<<std::endl;

    return weight;


}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr osm_pf_stereo::drop_zeros_s(sensor_msgs::PointCloud2 p_cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>) ;
    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
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


pcl::PointCloud<pcl::PointXYZRGB>::Ptr osm_pf_stereo::downsize_s(pcl::PointCloud<pcl::PointXYZRGB>::Ptr incloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> filter;
    filter.setInputCloud(incloud);
    filter.setLeafSize(down_sample_size,down_sample_size,down_sample_size);
    filter.filter(*p_cloud_filtered);
    // std::cout<<"\nFiltered!";

    return p_cloud_filtered;
}


std::vector<osm_pf_stereo::f> osm_pf_stereo::find_Wt_s(std::vector<pose> Xtbar,sensor_msgs::PointCloud2 p_cloud)
{
    // Preprocess PointCloud
    // auto start = std::chrono::high_resolution_clock::now();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud_ptr= drop_zeros_s(p_cloud);
    // auto stop = std::chrono::high_resolution_clock::now();

    // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start);
    // std::cout<<"\n Dropping zeros took :"<< duration.count() << " microseconds ";


    // start = std::chrono::high_resolution_clock::now();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud_filtered = downsize_s(p_cloud_ptr);
    // stop = std::chrono::high_resolution_clock::now();
    // duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start);
    // std::cout<<"\n Downsampling took :"<< duration.count() << " microseconds ";



    // std::cout<<"\nOutcloud size in function after  downsampling: "<<p_cloud_filtered->points.size()<<std::endl;
    // 
    std::vector<f> weights(Xtbar.size());

    // start = std::chrono::high_resolution_clock::now();
    for(int i=0;i<Xtbar.size();i++)
    {
        weights[i]=find_wt_s(Xtbar[i],p_cloud_filtered);
    }
    // stop = std::chrono::high_resolution_clock::now();

    // duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start);
    // std::cout<<"\n Total weighting took :"<< duration.count() << " microseconds ";


    return weights;
}
