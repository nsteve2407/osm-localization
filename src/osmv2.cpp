#include "osmv2.h"
#include "osm_pf.h"
#include<geometry_msgs/Pose2D.h>
#include<math.h>

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
    nh.getParam("/osm_particle_filter/path_to_dmat",path);
    
    ROS_INFO("Parameters Loaded Successfully");


    
    osmpf::osm_pf::f seed_x,seed_y;
    nh.getParam("/osm_particle_filter/seed_x",seed_x);
    nh.getParam("/osm_particle_filter/seed_y",seed_y);
    nh.getParam("/osm_particle_filter/pose_angular_res",pose_angular_res);
    nh.getParam("/osm_particle_filter/global_search_topX",global_search_topX);
    num_particles = int((360/pose_angular_res)*global_search_topX);
    
    global_search = nh.serviceClient<osm_localization::GlobalSearch>("/osm_global_search");
    osm_pf_core.reset(new osmpf::osm_pf(false,path,min_x,min_y,max_x,max_y,res_x,res_y,num_particles,seed_x,seed_y));
    
}

void osm_loc_v2::init_particles_from_srv(osm_localization::GlobalSearch::Response r)
{
    osmpf::pose p;
    for(int i=0;i<r.matches.size();i++)
    {
        for(osmpf::osm_pf::f j=0.0;j<360.0;j+=pose_angular_res)
        {
        p = osmpf::pose (osmpf::osm_pf::f(r.matches[i].x),osmpf::osm_pf::f(r.matches[i].y), osmpf::osm_pf::f((j*M_PI)/180.0));
        osm_pf_core->Xt.push_back(p);
        }
    }
    osm_pf_core->w_sum_sq = 1/(2*static_cast<osmpf::osm_pf::f>(osm_pf_core->Xt.size()));

    if(osm_pf_core->use_pi_weighting && osm_pf_core->use_pi_resampling)
    {
        osm_pf_core->Wt = std::vector<osmpf::osm_pf::f>(osm_pf_core->Xt.size(),1.0);
    }
    else if (osm_pf_core->use_pi_weighting && !osm_pf_core->use_pi_resampling)
    {
        osm_pf_core->Wt  = std::vector<osmpf::osm_pf::f>(osm_pf_core->Xt.size(),1.0);
    }
    else if(!osm_pf_core->use_pi_weighting && osm_pf_core->use_pi_resampling)
    
    {
        osm_pf_core->Wt  = std::vector<osmpf::osm_pf::f>(osm_pf_core->Xt.size(),1.0);
    }
    else
    {
        osm_pf_core->Wt  = std::vector<osmpf::osm_pf::f>(osm_pf_core->Xt.size(),0.0);
    }
    osm_pf_core->num_particles = osm_pf_core->Xt.size();

    osm_pf_core->w_sum_sq = 1/(3*static_cast<osmpf::osm_pf::f>(osm_pf_core->num_particles));
    std::cout<<"\n Weight sum sq initialized to: "<< osm_pf_core->w_sum_sq;
    std::cout<<"\n Neff initialized to: "<< 1/osm_pf_core->w_sum_sq;

    ROS_INFO("Particles intialized using Global Search");
}   

void osm_loc_v2::osm_v2_callback(const nav_msgs::OdometryConstPtr& odom,const sensor_msgs::PointCloud2ConstPtr& pc,const sensor_msgs::ImageConstPtr& lidar_bev_img)
{
    if(kidnapped)
    {
        osm_localization::GlobalSearch srv_msg;

        srv_msg.request.seed_x = osm_pf_core->init_x; //Need to update init_x, init_y
        srv_msg.request.seed_y = osm_pf_core->init_y;
        srv_msg.request.range  = osm_pf_core->init_cov_linear;
        srv_msg.request.lidar_bev_image = *lidar_bev_img;

        if( global_search.call(srv_msg.request,srv_msg.response))
        {
            ROS_INFO("Global Search completed");
            init_particles_from_srv(srv_msg.response);
            kidnapped=false;
            osm_pf_core->publish_msg(osm_pf_core->Xt,osm_pf_core->Wt,odom->header);
            // osm_pf_core->sync_v2.reset(new osmpf::osm_pf::Sync_v2(osmpf::osm_pf::sync_policy_osm_locv2(osm_pf_core->sync_queue_size),osm_pf_core->odom_sub,osm_pf_core->pc_sub,osm_pf_core->img_sub));
            // osm_pf_core->sync_v2->registerCallback(boost::bind(&osmpf::osm_pf::callback_v2_,osm_pf_core,_1,_2,_3));
            // ROS_INFO("Switch to local mode");
        }
        else
        {
            ROS_ERROR("Global search error");
        }

    }
    else
    {
        osm_pf_core->callback(odom,pc);
    }
}

void osm_loc_v2::attach_callback()
{
    osm_pf_core->sync_v2->registerCallback(boost::bind(&osm_loc_v2::osm_v2_callback,this,_1,_2,_3));
}

bool osm_loc_v2::is_kidnapped()
{
    return osm_pf_core->N_eff==0.0?true:false;
}
// Current error
// after service response all particles are to close/not uniformly distributed..try uniform/gaussian generator?