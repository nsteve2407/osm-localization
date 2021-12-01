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
    f min_x,min_y,max_x,max_y;
    nh.getParam("/osm_particle_filter/path_to_dmat",path);
    nh.getParam("/osm_particle_filter/min_x",min_x);
    nh.getParam("/osm_particle_filter/min_y",min_y);
    nh.getParam("/osm_particle_filter/max_x",max_x);
    nh.getParam("/osm_particle_filter/max_y",max_y);
    ROS_INFO("Parameters Loaded Successfully");

    ROS_INFO("Starting Particle Filter...");
    
    std::shared_ptr<osmpf::osm_pf> pf_ptr(new osmpf::osm_pf (path,min_x,min_y,max_x,max_y,500));
   
    // // Run Particle Filter
    pf_ptr->run();
    

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


    ros::spin();
    std::cout<<"Exiting Particle Filter..."<<std::endl;

    

    return 0;
    
}