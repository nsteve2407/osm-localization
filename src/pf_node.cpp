#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <xtensor.hpp>
#include <xtensor/xnpy.hpp>
#include <iostream>
#include "osm_pf.h"


typedef _Float64 f;


int main(int argc,char** argv)
{
    ROS_INFO("Initializing Particle Filter.....");
    ros::init(argc,argv,"Particle_Filtrer");
    ros::NodeHandle nh;
    ROS_INFO("Node created. Loading parameters.....");
    // // Values to access
    std::string path;
    f min_x,min_y,max_x,max_y,res_x,res_y;
    std::string sensing_mode;
    int resolution,num_particles;
    nh.getParam("/osm_particle_filter/path_to_dmat",path);
    nh.getParam("/osm_particle_filter/min_x",min_x);
    nh.getParam("/osm_particle_filter/min_y",min_y);
    nh.getParam("/osm_particle_filter/max_x",max_x);
    nh.getParam("/osm_particle_filter/max_y",max_y);
    nh.getParam("/osm_particle_filter/map_resolution",resolution);
    nh.getParam("/osm_particle_filter/num_particles",num_particles);
    nh.getParam("/osm_particle_filter/map_resolution_x",res_x);
    nh.getParam("/osm_particle_filter/map_resolution_y",res_y);
    nh.getParam("/osm_particle_filter/sensing_mode",sensing_mode);
    
    ROS_INFO("Parameters Loaded Successfully");

    ROS_INFO("Starting Particle Filter...");
    
    f seed_x,seed_y;
    nh.getParam("/osm_particle_filter/seed_x",seed_x);
    nh.getParam("/osm_particle_filter/seed_y",seed_y);

    
    std::shared_ptr<osmpf::osm_pf_stereo> pf_ptr_s;
    pf_ptr_s.reset(new osmpf::osm_pf_stereo (path,min_x,min_y,max_x,max_y,res_x,res_y,num_particles,seed_x,seed_y));

    if (sensing_mode=="lidar")
    {
        
        // pf_ptr_s.reset(new osmpf::osm_pf_stereo (path,min_x,min_y,max_x,max_y,res_x,res_y,num_particles,seed_x,seed_y));
        pf_ptr_s->run();
        
        // Run Particle Filter

    }
    else if(sensing_mode=="monocular")
    {
        pf_ptr_s.reset(new osmpf::osm_pf_stereo (path,min_x,min_y,max_x,max_y,res_x,res_y,num_particles,seed_x,seed_y,true));
        pf_ptr_s->run();
    }
    else
    {

        // pf_ptr_s.reset(new osmpf::osm_pf_stereo (path,min_x,min_y,max_x,max_y,res_x,res_y,num_particles,seed_x,seed_y));
        pf_ptr_s->run_s();
        // // Run Particle Filter
    

    }
    // if (sensing_mode=="lidar")
    // {
    //     pf_ptr->run();
    // }
    // else
    // {
    //     pf_ptr_s->run();
    //     pf_ptr.reset();
    // }
    
    // pf_ptr_s->run_s();
    
    
    

    // Arrayed saved in   n,e   /  lat,lon / y,x
    // int n = 0;
    // int e = 979;

    // cnpy::NpyArray d_mat = cnpy::npy_load("a.npy");
    
    // auto d_grid = xt::load_npy<_Float64>("/home/mkz/catkin_ws/src/osm-localization/distance_maps/d_grid.npy");
    // // xt::view(d_grid,xt::all(),2);
    // auto distance = d_grid(e,n);
    // std::cout<<distance<<std::endl;
    // auto c= d_grid(0,0)
    // printf(d_grid(0,0));

    while (ros::ok())
    {
        ros::spinOnce();
    }

    
    std::cout<<"Exiting Particle Filter..."<<std::endl;

    

    return 0;
    
}

// Current bug:: Initial selection going wrong..all points get roughly weight 1 only one has and then it selects the same one in the next step for all particles afer that particles diverge
// Consider starting at nodes?
// Also yaw needs to be bi directional? both sides?
//All weights come as 1?