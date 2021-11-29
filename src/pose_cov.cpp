#include<ros/ros.h>
#include<geometry_msgs/TwistStamped.h>
#include<geometry_msgs/TwistWithCovarianceStamped.h>
#include<boost/bind.hpp>
#include<boost/array.hpp>
class PoseCov
{
public:
ros::NodeHandle n;
ros::Publisher pub;
ros::Subscriber sub;

PoseCov();
void callback(const geometry_msgs::TwistStampedConstPtr& msg);
void run();

};

PoseCov::PoseCov()
{
    pub = n.advertise<geometry_msgs::TwistWithCovarianceStamped>("/filter/TwistWithCovariance",100);
    
}

void PoseCov::callback(const geometry_msgs::TwistStampedConstPtr& msg)
{
    geometry_msgs::TwistWithCovarianceStamped op_msg;

    op_msg.header = msg->header;
    op_msg.twist.twist = msg->twist;
    double cov_x = 0.01;
    double cov_y = 0.01;
    double cov_yaw = 0.01;
    op_msg.twist.covariance = boost::array<double,36>({
    cov_x, 0.,0.,0.,0.,0.,
    0.,cov_y,0.,0.,0.,0.,
    0.,0.,0.,0.,0.,0.,
    0.,0.,0.,0.,0.,0.,
    0.,0.,0.,0.,0.,0.,
    0.,0.,0.,0.,0.,cov_yaw});

    pub.publish(op_msg);
    // return op_msg;
}

void PoseCov::run()
{
    sub = n.subscribe<geometry_msgs::TwistStamped>("/vehicle/twist",1000,boost::bind(&PoseCov::callback,this,_1));
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"TiwstCovariance_converter");
    ros::NodeHandle nh;

    PoseCov convert_twist;
    convert_twist.run();

    while(ros::ok())
    {
        ros::spin();
    }

    return 0;
}