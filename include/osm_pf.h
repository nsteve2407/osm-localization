#include<ros/ros.h>
// #include<message_filters>
#include<message_filters/subscriber.h>
#include<message_filters/synchronizer.h>
#include<message_filters/sync_policies/approximate_time.h>
#include<nav_msgs/Odometry.h>
#include<sensor_msgs/PointCloud2.h>
#include <xtensor.hpp>
#include <xtensor/xnpy.hpp>
#include<string>
#include<vector>
#include<pcl-1.10/pcl/point_types.h>
#include<nav_msgs/Odometry.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include<std_msgs/Header.h>

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
        typedef _Float64 f;
        private:
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
        bool use_pi_weighting, use_pi_resampling,project_cloud,use_dynamic_resampling,estimate_gps_error;
        f w_sum_sq;
        int road_width,queue_size,sync_queue_size;
        f pi_gain;


        ros::NodeHandle nh;
        ros::Publisher pf_publisher;
        ros::Publisher pf_lat_lon;
        ros::Publisher pf_cloud_pub; 
        ros::Publisher pf_avg_pub;
        // ros::Publisher pf_pose;
        message_filters::Subscriber<nav_msgs::Odometry> odom_sub;
        message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub;
        typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry,sensor_msgs::PointCloud2> sync_policy;
        typedef message_filters::Synchronizer<sync_policy> Sync;
        std::shared_ptr<Sync> sync;
        int num_particles;
        // std::shared_ptr<std::vector<pose>> Xt;
        // std::shared_ptr<std::vector<f>> Wt;
        std::vector<pose> Xt;
        std::vector<f> Wt;

        nav_msgs::Odometry prev_odom;
        bool seed_set;
        f init_x;
        f init_y;
        public:
        // Methods
        osm_pf(std::string path_to_d_mat,f min_x,f min_y,f Max_x,f Max_y,f map_res_x,f map_res_y,int particles=100,f seed_x=0,f seed_y=0);
        void init_particles();
        pose find_xbar(pose x_tminus1,f dx,f dy, f dtheta);
        std::vector<pose> find_Xbar(std::vector<pose> X_tminus1,nav_msgs::Odometry odom);
        f find_wt(pose xbar,pcl::PointCloud<pcl::PointXYZI>::Ptr p_cloud_filtered);
        f find_wt_point(pcl::PointXYZI point);
        std::vector<f> find_Wt(std::vector<pose> Xtbar,sensor_msgs::PointCloud2 p_cloud);
        std::vector<pose> sample_xt(std::vector<pose> Xbar_t,std::vector<f>& Wt);
        void callback(const nav_msgs::OdometryConstPtr&,const sensor_msgs::PointCloud2ConstPtr&);
        pcl::PointCloud<pcl::PointXYZI>::Ptr drop_zeros(sensor_msgs::PointCloud2 p_cloud);
        void setSeed(f x,f y);
        pcl::PointCloud<pcl::PointXYZI>::Ptr downsize(pcl::PointCloud<pcl::PointXYZI>::Ptr);
        void run();
        std::shared_ptr<pose> weight_pose(std::vector<pose> Poses,std::vector<f> Weights);
        void publish_msg(std::vector<pose> X,std::vector<f> W,std_msgs::Header h);
    };
}