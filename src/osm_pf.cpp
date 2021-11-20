#include "osm_pf.h"

using namespace osmpf;

osm_pf::osm_pf(std::string path_to_d_mat,f min_x,f min_y,f Max_x,f Max_y,int particles)
{
    num_particles = particles;
    d_matrix = xt::load_npy<_Float64>(path_to_d_mat);
    origin_x = min_x;
    origin_y = min_y;
    max_x = Max_x;
    max_y = Max_y;
    pf_publisher = nh.advertise<geometry_msgs::Vector3>("oms_pose",100);
    odom_sub.subscribe(nh,"odometry_topic",100);
    pc_sub.subscribe(nh,"os_cloud_node/points",100);
    sync = new message_filters::Synchronizer<sync_policy>(sync_policy(10),odom_sub,pc_sub);
    xt::xarray<double>::shape_type shape = {1,(unsigned long)num_particles};
    Xt = new std::vector<pose>(num_particles);
    Wt = new std::vector<f>(num_particles);
}