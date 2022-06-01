#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/Image.h>
#include <pcl/common/transforms.h>
#include<pcl-1.10/pcl/point_types.h>
#include<nav_msgs/Odometry.h>
#include <pcl/point_types.h>
#include<pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include<cv_bridge/cv_bridge.h>
#include<Eigen/Dense>
#include<Eigen/Core>
#include <message_filters/subscriber.h>
#include "osm_pf.h"


class Img_to_cloud
{
    private:
    ros::NodeHandle nh;
    Eigen::ArrayXXf X_map_frame,Y_map_frame;
    message_filters::Subscriber<sensor_msgs::Image> img_sub;
    ros::Publisher pc_pub;
    int img_size_x, img_size_y;
    float scale_x, scale_y;
    public:
    Img_to_cloud();
    void Image_to_pcd_particleframe(const sensor_msgs::ImageConstPtr& image);
    void attachcallback();

};

Img_to_cloud::Img_to_cloud()
{

    nh.getParam("/osm_particle_filter/img_size_x",img_size_x);
    nh.getParam("/osm_particle_filter/img_size_y",img_size_y);
    nh.getParam("/osm_particle_filter/scale_x",scale_x);
    nh.getParam("/osm_particle_filter/scale_y",scale_y);

    img_sub.subscribe(nh,"/road_segment_image_top_view",100);
    pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/road_points",100);
    int image_c_x = img_size_x;
    int image_c_y = img_size_y*0.53;

    Eigen::MatrixXf P_x= Eigen::MatrixXf::Zero(img_size_x,img_size_y);
    Eigen::MatrixXf P_y= Eigen::MatrixXf::Zero(img_size_x,img_size_y);
    Eigen::MatrixXf C_x= Eigen::MatrixXf::Constant(img_size_x,img_size_y,image_c_x);
    Eigen::MatrixXf C_y= Eigen::MatrixXf::Constant(img_size_x,img_size_y,image_c_y);

    Eigen::RowVectorXf w = Eigen::RowVectorXf::LinSpaced(img_size_y,0,img_size_y-1);
    Eigen::VectorXf h = Eigen::VectorXf::LinSpaced(img_size_x,0,img_size_x-1);
    
    P_x.colwise() += h;
    P_y.rowwise() += w;
    // C_x += image_c_x;
    // C_y += image_c_y;
    X_map_frame = (C_x-P_x)*scale_x;
    Y_map_frame = (C_y-P_y)*scale_y;

    ROS_INFO("\nImage Matrices initialized");

}

void Img_to_cloud::Image_to_pcd_particleframe(const sensor_msgs::ImageConstPtr& image)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr op_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointXYZI point;

    cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(image,"bgr8");

    for(int i=0;i<image->height;i++)
    {
        for(int j=0;j<image->width;j++)
        {
            point.x = X_map_frame(i,j);
            point.y = Y_map_frame(i,j);
            point.z = 0.0;
            point.intensity = img_ptr->image.at<cv::Vec3b>(i,j)(1);
            op_cloud->points.push_back(point);
        }
    }

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*op_cloud,msg);
    msg.header = image->header;
    msg.header.frame_id = "osm_pose_estimate";
    pc_pub.publish(msg);


}

void Img_to_cloud::attachcallback()
{
    img_sub.registerCallback(boost::bind(&Img_to_cloud::Image_to_pcd_particleframe,this,_1));
}


int main(int argc,char** argv)
{
    ROS_INFO("Initializing Image to PointCloud conversion");
    ros::init(argc,argv,"img_to_pcd");
    ros::NodeHandle nh; 

    std::shared_ptr<Img_to_cloud> converter(new Img_to_cloud);
    converter->attachcallback();

    while (ros::ok())
    {
        ros::spinOnce();
    }
    

    return 0;
}