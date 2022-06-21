#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <xtensor.hpp>
#include <xtensor/xnpy.hpp>
#include <iostream>
#include "osmv2.h"



typedef _Float64 f;


int main(int argc,char** argv)
{
    ROS_INFO("Initializing OSM-Particle Filter V2");
    ros::init(argc,argv,"Particle_Filtrer");
    ros::NodeHandle nh;
    // // Values to access

    ROS_INFO("Starting Particle Filter");  

    
    std::string path;
    int global_search_topX;
    osmpf::osm_pf::f pose_angular_res; 

    osmpf::osm_pf::f min_x,min_y,max_x,max_y,res_x,res_y;
    std::string sensing_mode;
    int resolution,num_particles;
    nh.getParam("/osm_particle_filter/path_to_dmat",path);
    nh.getParam("/osm_particle_filter/min_x",min_x);
    nh.getParam("/osm_particle_filter/min_y",min_y);
    nh.getParam("/osm_particle_filter/max_x",max_x);
    nh.getParam("/osm_particle_filter/max_y",max_y);
    nh.getParam("/osm_particle_filter/map_resolution",resolution);

    nh.getParam("/osm_particle_filter/map_resolution_x",res_x);
    nh.getParam("/osm_particle_filter/map_resolution_y",res_y);
    nh.getParam("/osm_particle_filter/sensing_mode",sensing_mode);
    nh.getParam("/osm_particle_filter/path_to_dmat",path);
    
    ROS_INFO("Parameters Loaded Successfully");


    
    osmpf::osm_pf::f seed_x,seed_y;
    nh.getParam("/osm_particle_filter/seed_x",seed_x);
    nh.getParam("/osm_particle_filter/seed_y",seed_y);
    nh.getParam("/osm_particle_filter/pose_angular_res",pose_angular_res);
    nh.getParam("/osm_particle_filter/global_search_topX",global_search_topX);
    num_particles = int((360/pose_angular_res)*global_search_topX);
    

    std::shared_ptr<osmpf::osm_pf> pf_ptr(new osmpf::osm_pf(false,path,min_x,min_y,max_x,max_y,res_x,res_y,num_particles,seed_x,seed_y,global_search_topX,pose_angular_res));
    pf_ptr->attach_callback_v2();

    ROS_INFO("Particle Filter Initialized");

    while (ros::ok())
    {
        ros::spinOnce();
    }

    
    std::cout<<"Exiting Particle Filter"<<std::endl;

    

    return 0;
    
}

