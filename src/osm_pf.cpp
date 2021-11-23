#include "osm_pf.h"
#include<random>
#include<math.h>
#include<tf/transform_datatypes.h>

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
    cov_lin = (f)25;
    cov_angular = (f)M_PI_2;
    // Xt = std::make_shared<std::vector<pose>>();
    // Wt = std::make_shared<std::vector<f>>();
    Xt = std::vector<pose>(num_particles);
    Wt = std::vector<f>(num_particles);
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
        Xt[i]=p;
    }    
}

pose osm_pf::find_xbar(pose x_tminus1,nav_msgs::Odometry odom)
{
    f dx = odom.pose.pose.position.x;
    f dy = odom.pose.pose.position.y;
    f dtheta =  tf::getYaw(odom.pose.pose.orientation); //check sign converntions

    std::default_random_engine generator;
    std::normal_distribution<f> dist_x(x_tminus1.x+dx,cov_lin);
    std::normal_distribution<f> dist_y(x_tminus1.y+dy,cov_lin);
    std::normal_distribution<f> dist_theta(x_tminus1.theta+dtheta,cov_angular);
    pose p_k_bar(dist_x(generator),dist_y(generator),dist_theta(generator));
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
