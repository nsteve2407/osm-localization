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
#include<vector>


namespace osmpf
{
    class pose
    {
    public:
    _Float64 x,y;
    _Float64 theta;
    pose(_Float64 _x,_Float64 _y,_Float64 _theta){x = _x; y = _y; theta = _theta;}
    };

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
        message_filters::Synchronizer<sync_policy>* sync;
        int num_particles;
        // std::shared_ptr<std::vector<pose>> Xt;
        // std::shared_ptr<std::vector<f>> Wt;
        std::vector<pose> Xt;
        std::vector<f> Wt;
        f cov_lin, cov_angular;
        public:
        osm_pf(std::string path_to_d_mat,f min_x,f min_y,f Max_x,f Max_y,int particles=100);
        void init_particles();
        pose find_xbar(pose x_tminus1,nav_msgs::Odometry odom);
        std::vector<pose> find_Xbar(std::vector<pose> X_tminus1,nav_msgs::Odometry odom);
        f find_wt(f xbar,sensor_msgs::PointCloud2 p_cloud);
        f find_wt_point(float point);
        std::vector<f> find_Wt(std::vector<pose> Xtbar,sensor_msgs::PointCloud2 p_cloud);
        std::vector<pose> sample_xt(std::vector<pose> Xbar_t,std::vector<f> Wt);
    };
}