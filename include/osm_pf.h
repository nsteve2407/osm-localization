#pragma once
#include<ros/ros.h>
// #include<message_filters>
#include<message_filters/subscriber.h>
#include<message_filters/synchronizer.h>
#include<message_filters/sync_policies/approximate_time.h>
#include<nav_msgs/Odometry.h>
#include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/Image.h>
#include <xtensor.hpp>
#include <xtensor/xnpy.hpp>
#include<string>
#include<vector>
#include<pcl-1.10/pcl/point_types.h>
#include<nav_msgs/Odometry.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include<pcl/filters/random_sample.h>
#include <pcl/point_cloud.h>
#include<std_msgs/Header.h>
#include<Eigen/Dense>
#include<tf/transform_broadcaster.h>

namespace osmpf
{
    class pose
    {
    public:
    _Float64 x,y;
    _Float64 theta;
    pose(){x=0.0;y=0.0;theta=0.0;}
    pose(_Float64 _x,_Float64 _y,_Float64 _theta){x = _x; y = _y; theta = _theta;}
    };

    class osm_pf
    {
        public:
        typedef _Float64 f;
        // Attributes
        xt::xarray<f> d_matrix;
        float origin_x;
        float origin_y;
        float max_x;
        float max_y;
        float map_resolution_x,map_resolution_y;
        float down_sample_size;
        float init_cov_linear;
        float init_cov_angular;
        float odom_cov_lin;
        float odom_cov_angular;
        int count,resampling_count;
        bool use_pi_weighting, use_pi_resampling,project_cloud,use_dynamic_resampling,estimate_gps_error, adaptive_mode, mono_mode;
        f w_sum_sq,N_eff;//N_eff is a measure of number of effective particles. When weights are concentrated at a few particles N_eff falls less than N and when they are evenly distributed N_eff=N 
        int road_width,queue_size,sync_queue_size;
        float road_clloud_factor,non_road_cloud_factor;
        f pi_gain;
        std::string weight_function;


        ros::NodeHandle nh;
        ros::Publisher pf_publisher;
        ros::Publisher pf_lat_lon;
        ros::Publisher pf_cloud_pub; 
        ros::Publisher pf_avg_pub;
        // ros::Publisher pf_pose;
        message_filters::Subscriber<nav_msgs::Odometry> odom_sub;
        message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub;
        message_filters::Subscriber<sensor_msgs::Image> img_sub;
        typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry,sensor_msgs::PointCloud2> sync_policy;
        typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry,sensor_msgs::PointCloud2,sensor_msgs::Image> sync_policy_osm_locv2;
        typedef message_filters::Synchronizer<sync_policy> Sync;
        typedef message_filters::Synchronizer<sync_policy_osm_locv2> Sync_v2;
        std::shared_ptr<Sync> sync;
        std::shared_ptr<Sync_v2> sync_v2;
        tf::TransformBroadcaster osm_pose_broadcaster;
        pcl::PointCloud<pcl::PointXYZI>::Ptr p_cloud_filtered;

        int num_particles, min_particles, max_particles;
        float m, std_lim;
        double std_x,std_y;
        
        // std::shared_ptr<std::vector<pose>> Xt;
        // std::shared_ptr<std::vector<f>> Wt;
        std::vector<pose> Xt;
        std::vector<f> Wt;
        float avg_wt;

        nav_msgs::Odometry prev_odom;
        bool seed_set;
        f init_x;
        f init_y;
        // Random Generators
        // std::random_device rd;
        // std::mt19937 gen;
        std::shared_ptr<std::default_random_engine> gen; 
        std::shared_ptr<std::normal_distribution<f>> dist_ptr;
        std::normal_distribution<f> dist_x,dist_theta;
        // std::normal_distribution<f> dist_x;
        // Attributes for Monocular mode
        
        // Filters
        pcl::PassThrough<pcl::PointXYZI> road_filter;
        pcl::RandomSample<pcl::PointXYZI> random_sample;

        public:
        // Methods
        osm_pf(std::string path_to_d_mat,f min_x,f min_y,f Max_x,f Max_y,f map_res_x,f map_res_y,int particles=100,f seed_x=0,f seed_y=0,bool mono=false,float road_sampling_factor=0.70,float nroad__sampling_factor=0.2);
        osm_pf(bool v2,std::string path_to_d_mat,f min_x,f min_y,f Max_x,f Max_y,f map_res_x,f map_res_y,int particles=100,f seed_x=0,f seed_y=0,bool mono=false,float road_sampling_factor=0.70,float nroad__sampling_factor=0.2);
        void init_particles();
        pose find_xbar(pose x_tminus1,f dx,f dy, f dtheta);
        std::vector<pose> find_Xbar(std::vector<pose> X_tminus1,nav_msgs::Odometry odom);
        f find_wt(pose xbar,pcl::PointCloud<pcl::PointXYZI>::Ptr p_cloud_filtered);
        f find_wt_point(pcl::PointXYZI point);
        std::vector<f> find_Wt(std::vector<pose> Xtbar,sensor_msgs::PointCloud2 p_cloud);
        std::vector<pose> sample_xt(std::vector<pose> Xbar_t,std::vector<f>& Wt);
        void callback(const nav_msgs::OdometryConstPtr&,const sensor_msgs::PointCloud2ConstPtr&);
        void callback_v2_(const nav_msgs::OdometryConstPtr&,const sensor_msgs::PointCloud2ConstPtr&,const sensor_msgs::Image::ConstPtr img);
        pcl::PointCloud<pcl::PointXYZI>::Ptr drop_zeros(sensor_msgs::PointCloud2 p_cloud);
        void setSeed(f x,f y);
        pcl::PointCloud<pcl::PointXYZI>::Ptr downsize(pcl::PointCloud<pcl::PointXYZI>::Ptr);
        void run();
        std::shared_ptr<pose> weight_pose(std::vector<pose> Poses,std::vector<f> Weights);
        void publish_msg(std::vector<pose> X,std::vector<f> W,std_msgs::Header h);
        f weightfunction(f distance,f road_width,f intensity);
        std::vector<f> rescaleWeights(std::vector<f>& W); // Rescale weights to sum to 1.0
        void std_dibn();
        void update_num_particles();
        pcl::PointCloud<pcl::PointXYZI>::Ptr Image_to_pcd_particleframe(const sensor_msgs::Image& image,f pose_x,f pose_y,f pose_theta);
        void road_non_road_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr incloud,pcl::PointCloud<pcl::PointXYZI>::Ptr road_cloud,pcl::PointCloud<pcl::PointXYZI>::Ptr non_road_cloud);
    };

    class osm_pf_stereo: public osm_pf
    {
        public:
        typedef _Float64 f;
        osm_pf_stereo(std::string path_to_d_mat,f min_x,f min_y,f Max_x,f Max_y,f map_res_x,f map_res_y,int particles=100,f seed_x=0,f seed_y=0,bool mono=false);
        f find_wt_s(pose xbar,pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_cloud_filtered);
        f find_wt_point_s(pcl::PointXYZRGB point);
        std::vector<f> find_Wt_s(std::vector<pose> Xtbar,sensor_msgs::PointCloud2 p_cloud);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr drop_zeros_s(sensor_msgs::PointCloud2 p_cloud);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsize_s(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
        void callback_s(const nav_msgs::OdometryConstPtr&,const sensor_msgs::PointCloud2ConstPtr&);
        void run_s();
        void publish_msg_stereo(std::vector<pose> X,std::vector<f> W,std_msgs::Header h,const sensor_msgs::PointCloud2& ip_cloud);

        
    };
}