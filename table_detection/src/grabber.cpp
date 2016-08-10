#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <table_detection/ROIcloud.h>

#include <pcl_ros/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <boost/lexical_cast.hpp>


#define DBUG true
pcl::PointCloud<pcl::PointXYZ>::Ptr msg_cloud(new pcl::PointCloud<pcl::PointXYZ>());

sensor_msgs::PointCloud2 grabed_pc;
int srv_count;

void callback(const sensor_msgs::PointCloud2 &msg)
{
    if(DBUG)
    {
        pcl::fromROSMsg(msg, *msg_cloud);
    }
    grabed_pc = msg;
    sleep(1);
}

bool grab_pc2(table_detection::ROIcloud::Request &req, table_detection::ROIcloud::Response &res)
{
    res.pointcloud = grabed_pc;

    if(DBUG)
    {
        std::string name="/home/parallels/debug/grabber.pcd";
        std::string to_char= boost::lexical_cast<std::string>(srv_count);
        name.insert(name.length()-4, to_char);
        pcl::io::savePCDFileASCII(name, *msg_cloud);
        srv_count++;
    }
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "triggered_grabber");

    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/head_xtion/depth/points", 1, callback);
    ros::ServiceServer server = nh.advertiseService("ROIcloud", grab_pc2);
    srv_count = 0;

    ros::spin();
}
