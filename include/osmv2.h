#include "osm_pf.h"
#include<osm_localization/GlobalSearch.h>
#include<nav_msgs/Odometry.h>
#include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/Image.h>

class osm_loc_v2 
{
    private:
    // Attributes
    bool kidnapped;
    std::shared_ptr<osmpf::osm_pf> osm_pf_core;
    int global_search_topX;
    int pose_angular_res; 
    ros::ServiceClient global_search;
    public:
    // Methods
    osm_loc_v2();
    void init_particles_from_srv(osm_localization::GlobalSearch::Response r);
    bool is_kidnapped();
    void osm_v2_callback(const nav_msgs::OdometryConstPtr&,const sensor_msgs::PointCloud2ConstPtr&,const sensor_msgs::ImageConstPtr&);
    void attach_callback();
};