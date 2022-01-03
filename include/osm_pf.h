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
        float map_resolution;
        int count;
        ros::NodeHandle nh;
        ros::Publisher pf_publisher;
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
        f cov_lin, cov_angular;
        nav_msgs::Odometry prev_odom;
        bool seed_set;
        f init_x;
        f init_y;
        public:
        // Methods
        osm_pf(std::string path_to_d_mat,f min_x,f min_y,f Max_x,f Max_y,f map_res,int particles=100,f seed_x=0,f seed_y=0);
        void init_particles();
        pose find_xbar(pose x_tminus1,nav_msgs::Odometry odom);
        std::vector<pose> find_Xbar(std::vector<pose> X_tminus1,nav_msgs::Odometry odom);
        f find_wt(pose xbar,sensor_msgs::PointCloud2 p_cloud);
        f find_wt_point(pcl::PointXYZI point);
        std::vector<f> find_Wt(std::vector<pose> Xtbar,sensor_msgs::PointCloud2 p_cloud);
        std::vector<pose> sample_xt(std::vector<pose> Xbar_t,std::vector<f>& Wt);
        void callback(const nav_msgs::OdometryConstPtr&,const sensor_msgs::PointCloud2ConstPtr&);
        pcl::PointCloud<pcl::PointXYZI>::Ptr drop_zeros(sensor_msgs::PointCloud2 p_cloud);
        void setSeed(f x,f y);
        pcl::PointCloud<pcl::PointXYZI>::Ptr downsize(pcl::PointCloud<pcl::PointXYZI>::Ptr);
        void run();
    };
}