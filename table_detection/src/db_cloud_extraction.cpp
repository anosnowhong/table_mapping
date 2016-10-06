#include <mongodb_store/message_store.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <table_detection/db_extract.h>
#include <table_detection/db_extract_whole_table.h>
#include <table_detection/db_table_clouds.h>
#include <table_detection/db_table.h>
#include <std_msgs/Int32.h>
#include "table_detection/table_tool.h"

/* #A backend node that does the extraction work with MongoDB.
 * Load point cloud data from global_cloud collection.
 * call the extraction service
 * use table parameter to filer out table plane
 * store to table_plane collection
 * */

ros::NodeHandlePtr nh;

int checked_num=0;
float normal_threshold;
float search_radius;
int neighbour_required;
int statistical_knn;
float std_dev_dist;

bool extract(table_detection::db_extract::Request &req, table_detection::db_extract::Response &res)
{
    ROS_INFO("Starting extract table...");
    Table<pcl::PointXYZ> ex_tb(nh, normal_threshold);
    //extract table planes and store in collection 'table_clouds'
    ex_tb.tb_detection("table_clouds");
    //TODO: Redefine the msg, make mongodb efficient work for a long time.
    //TODO: Support for static data.
    ROS_INFO("Done table extraction!");
}

bool extract_whole_table(table_detection::db_extract_whole_table::Request &req, table_detection::db_extract_whole_table::Response &res)
{
    ROS_INFO("Starting extract whole table");
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "db_cloud_extraction");
    nh.reset(new ros::NodeHandle);
    ros::NodeHandle pn("~");
    pn.param<float>("normal_threshold", normal_threshold, 15.0);
    //filter1
    pn.param<float>("search_radius", search_radius, 0.8);
    pn.param<int>("neighbour_required", neighbour_required, 20);
    //filter2
    pn.param<int>("statistical_knn", statistical_knn, 50);
    pn.param<float>("std_dev_dist", std_dev_dist, 1.0);

    //table parts
    ros::ServiceServer ext_srv = nh->advertiseService("db_extract", extract);
    //more complete table
    ros::ServiceServer ext_srv2 = nh->advertiseService("db_extract_whole_table", extract_whole_table);

    ros::spin();
}