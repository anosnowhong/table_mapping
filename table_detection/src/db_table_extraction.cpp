#include <ros/ros.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_normal_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <mongodb_store/message_store.h>
#include <table_detection/db_table_clouds.h>
#include <pcl/features/normal_3d.h>


typedef pcl::PointXYZ  Point;
typedef pcl::PointCloud<Point> pcl_cloud;
ros::NodeHandlePtr nh;

bool extract_table(pcl_cloud::Ptr cloud)
{

    pcl_cloud::Ptr cloud1(new pcl_cloud());
    pcl_cloud::Ptr cloud_out(new pcl_cloud());

    pcl::PassThrough<Point> pass;
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.45,1.2);
    pass.setInputCloud(cloud);
    pass.filter(*cloud1);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    //pcl::Normal nor;
    //nor.normal_x=0.0;
    //nor.normal_y=0.0;
    //nor.normal_z=1.0;
    //pcl::SampleConsensusModelNormalPlane<Point, pcl::Normal> normal_sac();

    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (cloud1);
    seg.segment (*inliers, *coefficients);

    if(inliers->indices.size () == 0){
        ROS_INFO("No plane detected!");
        return false;
    }

    //form a new cloud from indices
    pcl::ExtractIndices<Point> extract;
    extract.setInputCloud (cloud1);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_out);
    //pcl::io::savePCDFileASCII("/home/parallels/debug/test.pcd", *cloud_out);

    //normal estimation and filter table plane
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud_out);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch (0.03);
    ne.compute (*cloud_normals);

    pcl::Normal nor;
    float size = 0.0;
    for(int i=0;i<cloud_normals->height*cloud_normals->width;i++){

        if(isnan(cloud_normals->at(i).normal_x)){
            //std::cout<<cloud_normals->at(i).normal_x<<std::endl;
            }
        else{
            nor.normal_x += cloud_normals->at(i).normal_x;
            nor.normal_y += cloud_normals->at(i).normal_y;
            nor.normal_z += cloud_normals->at(i).normal_z;
            size++;
        }
    }
    //std::cout<<size<<std::endl;
    nor.normal_x = nor.normal_x/size;
    nor.normal_y = nor.normal_x/size;
    nor.normal_z = nor.normal_x/size;

    std::cout<<nor.normal_x<<std::endl;
    std::cout<<nor.normal_y<<std::endl;
    std::cout<<nor.normal_z<<std::endl;

    if(nor.normal_x>0.1||nor.normal_y>0.1||nor.normal_z>0.1)
        return false;


    //conversion
    sensor_msgs::PointCloud2 table_cloud;
    pcl::toROSMsg(*cloud_out, table_cloud);

    //store to mongodb
    mongodb_store::MessageStoreProxy messageStore(*nh,"whole_table_clouds");
    messageStore.insert(table_cloud);

}


bool extract(table_detection::db_table_clouds::Request &req, table_detection::db_table_clouds::Response &res)
{
    //load clouds form icp_clouds
    mongodb_store::MessageStoreProxy messageStore(*nh,"icp_clouds");
    std::vector< boost::shared_ptr<sensor_msgs::PointCloud2> > result_pc2;
    //search all point clouds
    messageStore.query<sensor_msgs::PointCloud2>(result_pc2);

    pcl_cloud::Ptr loaded_cloud;
    for(int i=0;i<result_pc2.size();i++){

        loaded_cloud.reset(new pcl_cloud());
        pcl::fromROSMsg(*result_pc2[i], *loaded_cloud);
        extract_table(loaded_cloud);

    }

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "db_cloud_extraction");
    nh.reset(new ros::NodeHandle);
    ros::NodeHandle pn("~");

    ros::ServiceServer table_srv = nh->advertiseService("db_table_clouds", extract);

    ros::spin();

}