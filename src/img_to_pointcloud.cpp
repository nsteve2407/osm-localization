#include<ros/ros.h>
#include <pcl/common/transforms.h>
#include<pcl-1.10/pcl/point_types.h>
#include<nav_msgs/Odometry.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>

pcl::PointCloud<pcl::PointXYZI>::Ptr osm_pf::Image_to_pcd_particleframe(const sensor_msgs::Image& image,f pose_x,f pose_y,f pose_theta)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr op_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointXYZI point;

    // Eigen::ArrayXXf X_map_frame = L_x*sin(pose_theta)+L_y*cos(pose_theta)+pose_x;
    // Eigen::ArrayXXf Y_map_frame = L_y*sin(pose_theta)-L_x*cos(pose_theta)+pose_y;
    Eigen::ArrayXXf X_map_frame = L_x;
    Eigen::ArrayXXf Y_map_frame = L_y;

    cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(image,"bgr8");

    for(int i=0;i<image.height;i++)
    {
        for(int j=0;j<image.width;j++)
        {
            point.x = X_map_frame(i,j);
            point.y = Y_map_frame(i,j);
            point.z = 0.0;
            point.intensity = img_ptr->image.at<cv::Vec3b>(i,j)(1);
            op_cloud->points.push_back(point);
        }
    }


    return op_cloud;
}