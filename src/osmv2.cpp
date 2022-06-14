#include "osmv2.h"
#include "osm_pf.h"
#include<geometry_msgs/Pose2D.h>

osm_loc_v2::osm_loc_v2()
{
    kidnapped = true;
    ros::NodeHandle nh;

    // // Values to access
    std::string path;
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

    nh.getParam("/osm_particle_filter/pose_angular_res",pose_angular_res);
    nh.getParam("/osm_particle_filter/path_to_dmat",path);
    
    ROS_INFO("Parameters Loaded Successfully");


    
    osmpf::osm_pf::f seed_x,seed_y;
    nh.getParam("/osm_particle_filter/pose_angular_res",pose_angular_res);
    nh.getParam("/osm_particle_filter/global_search_topX",global_search_topX);
    num_particles = int((360/pose_angular_res)*global_search_topX);
    
    osm_pf_core.reset(new osmpf::osm_pf(false,path,min_x,min_y,max_x,max_y,res_x,res_y,num_particles,seed_x,seed_y));
    
}

void osm_loc_v2::init_particles_from_srv(osm_localization::GlobalSearch::Response r)
{
    

}