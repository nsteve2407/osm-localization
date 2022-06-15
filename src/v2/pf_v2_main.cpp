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

    std::shared_ptr<osm_loc_v2> pf_ptr(new osm_loc_v2);
    ROS_INFO("Particle Filter Initialized");
    pf_ptr->attach_callback();


    while (ros::ok())
    {
        ros::spinOnce();
    }

    
    std::cout<<"Exiting Particle Filter"<<std::endl;

    

    return 0;
    
}

