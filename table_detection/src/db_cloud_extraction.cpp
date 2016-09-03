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

/* #A backend node that does the extraction work with MongoDB.
 * Load point cloud data from global_cloud collection.
 * call the extraction service
 * use table parameter to filer out table plane
 * store to table_plane collection
 * */

ros::ServiceClient table_client;
ros::ServiceClient table_client2;
ros::NodeHandlePtr nh;

int checked_num=0;

#define Debug false

//store strands_perception_msg to DB
bool extract(table_detection::db_extract::Request &req, table_detection::db_extract::Response &res)
{
    ROS_INFO("Starting extract table");
    //query point cloud and tf
    mongodb_store::MessageStoreProxy global_clouds(*nh,"global_clouds");
    //store point cloud that been transformed
    mongodb_store::MessageStoreProxy table_store(*nh,"table_planes");
    std::vector< boost::shared_ptr<sensor_msgs::PointCloud2> > result_pc2;


    //search all point clouds
    global_clouds.query<sensor_msgs::PointCloud2>(result_pc2);

    //load check_num form mongodb, this value record how many global clouds has been checked.
    std::vector< boost::shared_ptr<std_msgs::Int32> > checked_amount;
    table_store.queryNamed<std_msgs::Int32>("checked_num", checked_amount);
    if(checked_amount.size() == 0){
        std_msgs::Int32 tmp;
        tmp.data = checked_num;
        table_store.insertNamed("checked_num",tmp);
    }
    else{
        //load record form mongodb
        checked_num = checked_amount[0]->data;
    }

    ROS_INFO("Cloud found: %lu, already checked: %d", result_pc2.size(), checked_num);

    for(;checked_num<result_pc2.size();checked_num++)
    {
        //call the extraction service, check if there is a table in the cloud
        table_client.waitForExistence();
        table_detection::db_table req;
        req.request.cloud = *result_pc2[checked_num];
        bool rc = table_client.call(req);

        //response with index of cloud that may contain a table plane
        if(rc)
            res.cloud_has_plane.push_back(checked_num);
    }
    //update check_num in mongodb
    std_msgs::Int32 tmp;
    tmp.data = checked_num;
    table_store.updateNamed("checked_num",tmp);

    if(Debug)
    {
        for(int i=0;i<res.cloud_has_plane.size();i++)
            std::cout<<res.cloud_has_plane[i]<<std::endl;
    }
    return true;
}

//store the point cloud of table
bool extract_whole_table(table_detection::db_extract_whole_table::Request &req, table_detection::db_extract_whole_table::Response &res)
{
    ROS_INFO("Starting extract whole table");

    while(!table_client2.waitForExistence())
        ROS_INFO("waiting for db_table_clouds service");
    table_detection::db_table_clouds cc;
    cc.request.cloud = req.cloud;
    bool rc = table_client2.call(cc);
    if(!rc)
        ROS_INFO("db_table_clouds service return FALSE!!!");

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "db_cloud_extraction");
    nh.reset(new ros::NodeHandle);
    ros::NodeHandle pn("~");

    table_client=nh->serviceClient<table_detection::db_table>("db_table");
    table_client2=nh->serviceClient<table_detection::db_table_clouds>("db_table_clouds");
    //table parts
    ros::ServiceServer ext_srv = nh->advertiseService("db_extract", extract);
    //more complete table
    ros::ServiceServer ext_srv2 = nh->advertiseService("db_extract_whole_table", extract_whole_table);

    ros::spin();
}