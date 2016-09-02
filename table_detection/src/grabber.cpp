#include <ros/ros.h>
//#include <boost/foreach.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <table_detection/ROIcloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>
//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <boost/lexical_cast.hpp>

#define DBUG false

typedef pcl::PointXYZ  Point;
typedef pcl::PointCloud<Point> pcl_cloud;

pcl_cloud::Ptr msg_cloud(new pcl_cloud());
pcl_cloud::Ptr short_range_cloud(new pcl_cloud());

ros::Publisher pub;
boost::mutex cloud_mutex;
float min_distance, max_distance;

sensor_msgs::PointCloud2 grabed_pc;
int srv_count;

void callback(const sensor_msgs::PointCloud2 &msg)
{
    pcl::fromROSMsg(msg, *msg_cloud);
    //filter out point cloud that is too far away
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setFilterFieldName("z");
    pass.setFilterLimits(min_distance,max_distance);
    pass.setInputCloud(msg_cloud);
    pass.filter(*short_range_cloud);

    sensor_msgs::PointCloud2 new_msg;
    pcl::toROSMsg(*short_range_cloud,new_msg);

    if(DBUG)
    {
        pcl::fromROSMsg(msg, *msg_cloud);
    }
    grabed_pc = new_msg;
    //special condition: force point cloud from morse to false as they does contain nan point
    grabed_pc.is_dense=false;

    usleep(100000);
}

bool grab_pc2(table_detection::ROIcloud::Request &req, table_detection::ROIcloud::Response &res)
{
    //cloud_mutex.try_lock();
    res.pointcloud = grabed_pc;

    if(DBUG)
    {
        std::string name="/home/parallels/debug/grabber.pcd";
        std::string to_char= boost::lexical_cast<std::string>(srv_count);
        name.insert(name.length()-4, to_char);
        pcl::io::savePCDFileASCII(name, *msg_cloud);
        srv_count++;
    }
    pub.publish(grabed_pc);
    //cloud_mutex.unlock();
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "triggered_grabber");

    ros::NodeHandle nh;
    ros::NodeHandle pn("~");

    ros::Subscriber sub = nh.subscribe("/head_xtion/depth/points", 1, callback);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud/tomongo", 1);
    pn.param<float>("min_distance",min_distance, 0.0);
    pn.param<float>("max_distance",max_distance, 3.0);

    ros::ServiceServer server = nh.advertiseService("ROIcloud", grab_pc2);
    srv_count = 0;

    ros::spin();
}
