#include<ros/ros.h>
#include<message_filter.h>
#include<message_filters/subscriber.h>
#include<message_filters/synchronizer.h>
#include<message_filters/sync_policies/approximate_time.h>
#include<nav_msgs/Odometry.h>
#include<sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include <xtensor.hpp>
#include <xtensor/xnpy.hpp>
#include<string>


namespace osmpf
{
    class osm_pf
    {
        typedef _Float64 f;
        private:
        xt::xarray<f> d_matrix;
        f origin_x;
        f origin_y;
        f max_x;
        f max_y;
        ros::NodeHandle nh;
        ros::Publisher pf_publisher;
        message_filters::Subscriber<nav_msgs::Odometry> odom_sub;
        message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub;
        typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry,sensor_msgs::PointCloud2> sync_policy;
        message_filters::Synchronizer<sync_policy> sync;
        int num_particles;
        xt::xarray<f> Xt;
        xt::xarray<f> Wt;
        std::string d_mat_path;
        public:
        osm_pf(std::string path_to_d_mat);
        xt::xarray<f> sample_pose();
        f find_xbar(f x_tminus1,nav_msgs::Odometry odom);
        xt::xarray<f> find_Xbar(xt::xarray<f> X_tminus1,nav_msgs::Odometry odom);
        f find_wt(f xbar,sensor_msgs::PointCloud2 p_cloud);
        f find_wt_x(float point);
        xt::xarray<f> find_Wt(xt::xarray<f> Xtbar,sensor_msgs::PointCloud2 p_cloud);
        xt::xarray<f> sample_xt(xt::xarray<f> Xbar_t,xt::xarray<f> Wt);
    };
}