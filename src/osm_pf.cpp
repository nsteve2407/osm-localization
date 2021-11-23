#include "osm_pf.h"
#include<random>
#include<math.h>

using namespace osmpf;

osm_pf::osm_pf(std::string path_to_d_mat,f min_x,f min_y,f Max_x,f Max_y,int particles)
{
    num_particles = particles;
    d_matrix = xt::load_npy<f>(path_to_d_mat);
    origin_x = min_x;
    origin_y = min_y;
    max_x = Max_x;
    max_y = Max_y;
    pf_publisher = nh.advertise<geometry_msgs::Vector3>("oms_pose",100);
    odom_sub.subscribe(nh,"odometry_topic",100);
    pc_sub.subscribe(nh,"os_cloud_node/points",100);
    sync = new message_filters::Synchronizer<sync_policy>(sync_policy(10),odom_sub,pc_sub);
    
    Xt = std::make_shared<std::vector<pose>>();
    Wt = std::make_shared<std::vector<f>>();
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
        Xt->emplace_back(p);
    }    
}