#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <xtensor.hpp>
#include <xtensor/xnpy.hpp>
#include <iostream>














int main(int argc,char** argv)
{
    ros::init(argc,argv,"Particle_Filtrer");
    ros::NodeHandle ng;

    // Values to access

    // Arrayed saved in   n,e   /  lat,lon / y,x
    int n = 0;
    int e = 979;

    // cnpy::NpyArray d_mat = cnpy::npy_load("a.npy");
    
    auto d_grid = xt::load_npy<_Float64>("/home/mkz/catkin_ws/src/osm-localization/distance_maps/d_grid.npy");
    // xt::view(d_grid,xt::all(),2);
    auto distance = d_grid(e,n);
    std::cout<<distance<<std::endl;
    // auto c= d_grid(0,0)
    // printf(d_grid(0,0));

    return 0;
    
}