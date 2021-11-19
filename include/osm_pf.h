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
        private:
        xt::xarray<_Float64> d_matrix;
        _Float64 origin_x;
        _Float64 origin_y;
        _Float64 max_x;
        _Float64 max_y;
        ros::NodeHandle nh;
        ros::Publisher pf_publisher;
        message_filters::Subscriber<nav_msgs::Odometry> odom_sub;
        message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub;
        typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry,sensor_msgs::PointCloud2> sync_policy;
        message_filters::Synchronizer<sync_policy> sync;
        int num_particles;
        xt::xarray<_Float64> X_t;
        xt::xarray<_Float64> W_t;
        std::string d_mat_path;
        public:

    };
}