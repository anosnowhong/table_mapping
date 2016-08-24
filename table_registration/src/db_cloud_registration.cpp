#include <mongodb_store/message_store.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include "table_registration/registration_operator.hpp"
#include <table_registration/ToGlobal.h>
#include <sensor_msgs/PointCloud2.h>
#include <table_registration/ICP_Cloud.h>
#include <table_detection/db_extract.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <table_detection/db_extract_whole_table.h>


/* #A backend node that does the registration work with MongoDB.
 * Load point cloud and tf data from ws_observation collection.
 * Convert to global coordinate system (/map)
 * Stored to a new collection.
 * */
typedef boost::shared_ptr<geometry_msgs::TransformStamped>  TransformStampedPtr;
typedef boost::shared_ptr<sensor_msgs::PointCloud2> PointCloud2Ptr;

#define Debug false

ros::NodeHandlePtr nh;
int pan_interval = 30;
ros::ServiceClient extract_client;
ros::ServiceClient extract_client2;

//service call to convert cloud in ws_observation to global_cloud collection
bool to_global(table_registration::ToGlobal::Request &req, table_registration::ToGlobal::Response &res)
{
    //query point cloud and tf
    mongodb_store::MessageStoreProxy messageStore(*nh,"ws_observations");
    //store point cloud that been transformed
    mongodb_store::MessageStoreProxy cloud_store(*nh,"global_clouds");

    std::vector< boost::shared_ptr<sensor_msgs::PointCloud2> > result_pc2;
    std::vector< boost::shared_ptr<sensor_msgs::PointCloud2> > result_global_pc2;
    std::vector< boost::shared_ptr<geometry_msgs::TransformStamped> > result_tf;

    //search all point clouds
    messageStore.query<sensor_msgs::PointCloud2>(result_pc2);
    //get all the tf transformation
    messageStore.query<geometry_msgs::TransformStamped>(result_tf);
    //get all transformed point clouds
    cloud_store.query<sensor_msgs::PointCloud2>(result_global_pc2);

    //how many clouds need to be transformed
    int queue = result_pc2.size()-result_global_pc2.size();
    int stored_num = result_pc2.size();
    if (queue == 0){
        ROS_INFO("Already transformed!");
    }

    ROS_INFO("pc2 amount: %lu tf amount: %lu",result_pc2.size(), result_tf.size());
    ROS_INFO("Remaining To Transform: %d",queue);

    pcl::PointCloud<pcl::PointXYZ>::Ptr db_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr db_store(new pcl::PointCloud<pcl::PointXYZ>());
    sensor_msgs::PointCloud2 db_store_msg;
    geometry_msgs::Vector3 trans;
    geometry_msgs::Quaternion qua;
    for(int i=0 ;i<queue;i++)
    {
        //reset
        db_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        db_store.reset(new pcl::PointCloud<pcl::PointXYZ>);

        trans = result_tf[i+stored_num-queue]->transform.translation;
        qua = result_tf[i+stored_num-queue]->transform.rotation;
        //load point cloud data to pcl::PointCloud2
        pcl::fromROSMsg(*result_pc2[i+stored_num-queue],*db_cloud);
        //do the transformation things
        registration_operator<pcl::PointXYZ> reg_op(db_cloud,trans,qua);
        reg_op.to_global(*db_store);

        if(Debug){
            std::string name="/home/parallels/debug/.pcd";
            std::stringstream ss;
            ss<< i;
            std::string str = ss.str();
            name.insert(name.length()-4, str);
            std::cout<<name<<std::endl;
            pcl::io::savePCDFileASCII(name, *db_store);
        }

        //convert to msg and store it
        pcl::toROSMsg(*db_store,db_store_msg);

        //keep same header but different frame_id
        db_store_msg.header=result_pc2[i+stored_num-queue]->header;
        db_store_msg.header.frame_id = "/map";
        cloud_store.insert(db_store_msg);
    }
    ROS_INFO("Done. Point Clouds has been transformed to global coordinate '/map'");

    //extract table plane from global clouds, record index as well
    extract_client.waitForExistence();
    table_detection::db_extract to_extract;
    extract_client.call(to_extract);
    int plane_num = to_extract.response.cloud_has_plane.size();
    ROS_INFO("Detect %d possible table planes", plane_num);
    std::vector<int> plane_index;
    for(int i=0;i< plane_num;i++)
        plane_index.push_back(to_extract.response.cloud_has_plane[i]);

    //recorde the index of point cloud that can extract a plane
    //use the index to do accurate icp by using the nearby clouds
    mongodb_store::MessageStoreProxy icp_cloud(*nh,"icp_clouds");
    registration_operator<pcl::PointXYZ> reg_icp;
    //requery mongodb get latest result
    result_global_pc2.clear();
    cloud_store.query<sensor_msgs::PointCloud2>(result_global_pc2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr tfed_cloud1(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr tfed_cloud2(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sum(new pcl::PointCloud<pcl::PointXYZ>());

    int idd = 360/pan_interval;
    for(int i=0;i<plane_num;i++)
    {
        cloud1.reset(new pcl::PointCloud<pcl::PointXYZ>());
        cloud2.reset(new pcl::PointCloud<pcl::PointXYZ>());
        tfed_cloud1.reset(new pcl::PointCloud<pcl::PointXYZ>());
        tfed_cloud2.reset(new pcl::PointCloud<pcl::PointXYZ>());
        cloud_sum.reset(new pcl::PointCloud<pcl::PointXYZ>());
        table_registration::ICP_Cloud whole_table_with_cloud_index;

        ROS_INFO("Looking for cloud index %d, and nearby clouds", plane_index[i]);
        //the cloud before the cloud that contains plane
        if(plane_index[i]%idd == 0){
            pcl::fromROSMsg(*result_global_pc2[plane_index[i]],*cloud1);
            whole_table_with_cloud_index.index.push_back(plane_index[i]);
        }
        else{
            pcl::fromROSMsg(*result_global_pc2[plane_index[i]-1],*cloud1);
            whole_table_with_cloud_index.index.push_back(plane_index[i]-1);
        }
        pcl::fromROSMsg(*result_global_pc2[plane_index[i]],*cloud2);
        whole_table_with_cloud_index.index.push_back(plane_index[i]);

        *cloud_sum += *cloud1;
        //get the transform between 2 clouds
        Eigen::Matrix4f mat1 = reg_icp.accurate_icp(cloud1, cloud2, tfed_cloud1);
        pcl::transformPointCloud(*cloud2, *tfed_cloud1, mat1);
        *cloud_sum += *tfed_cloud1;

        //the cloud after the cloud that contains plane
        tfed_cloud1.reset(new pcl::PointCloud<pcl::PointXYZ>());
        cloud1 = cloud2;
        if(plane_index[i]%idd == idd-1){
            pcl::fromROSMsg(*result_global_pc2[plane_index[i]],*cloud2);
            whole_table_with_cloud_index.index.push_back(plane_index[i]);
        }
        else{
            pcl::fromROSMsg(*result_global_pc2[plane_index[i]+1],*cloud2);
            whole_table_with_cloud_index.index.push_back(plane_index[i]+1);
        }
        Eigen::Matrix4f mat2 = reg_icp.accurate_icp(cloud1, cloud2, tfed_cloud1);
        //2 times transformation
        pcl::transformPointCloud(*cloud2, *tfed_cloud1, mat2);
        pcl::transformPointCloud(*tfed_cloud1, *tfed_cloud2, mat1);

        *cloud_sum += *tfed_cloud2;

        //call extraction service and pass the combined three clouds
        table_detection::db_extract_whole_table table_req;
        sensor_msgs::PointCloud2 whole_table;
        pcl::toROSMsg(*cloud_sum, whole_table);
        whole_table.header.frame_id="/map";
        whole_table_with_cloud_index.icp_cloud=whole_table;

        icp_cloud.insert(whole_table_with_cloud_index);
        ROS_INFO("Done. ICP registered cloud has been stored to icp_cloud collection. ");

        //extract table cloud from icp cloud
        while(!extract_client2.waitForExistence())
            ROS_INFO("Waiting for service db_extract_whole_table to be available");
        table_req.request.cloud=whole_table;
        bool rc = extract_client2.call(table_req);
        if(!rc)
            ROS_INFO("Whole Table Extraction Service return FALSE!!!");

    }

    ROS_INFO("Done. Whole Table has been extracted. ");


    return true;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "db_cloud_registration");
    nh.reset(new ros::NodeHandle);
    ros::NodeHandle pn("~");
    pn.param<int>("pan_interval", pan_interval, 30);

    ros::ServiceServer service = nh->advertiseService("to_global",to_global);
    extract_client = nh->serviceClient<table_detection::db_extract>("db_extract");
    extract_client2 = nh->serviceClient<table_detection::db_extract_whole_table>("db_extract_whole_table");

    ros::spin();
}
