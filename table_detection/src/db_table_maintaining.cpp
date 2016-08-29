#include <ros/ros.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <mongodb_store/message_store.h>
#include <table_detection/db_table_clouds.h>
#include <table_detection/db_table.h>
#include <pcl/features/normal_3d.h>
#include <strands_perception_msgs/Table.h>
#include <pcl/surface/concave_hull.h>
#include <std_msgs/Int32.h>


#define Debug true
typedef pcl::PointXYZ  Point;
typedef pcl::PointCloud<Point> pcl_cloud;
ros::NodeHandlePtr nh;
int checked_num=0;

void extract_convex(pcl_cloud::Ptr cloud_in,
                    pcl::PointIndices::Ptr inliers,
                    pcl::ModelCoefficients::Ptr coefficients,
                    pcl_cloud::Ptr cloud_out)
{
    pcl_cloud::Ptr projected(new pcl_cloud());

    pcl::ProjectInliers<Point> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setIndices (inliers);
    proj.setInputCloud (cloud_in);
    proj.setModelCoefficients (coefficients);
    proj.filter (*projected);

    pcl::ConcaveHull<Point> chull;
    chull.setInputCloud (projected);
    chull.setAlpha (0.1);
    chull.reconstruct (*cloud_out);
}

//2d binding box for table shape
void binding_box()
{

}
//loading msg form collection and merge tables that may linked together
bool merge_table_msg()
{
    //load table shapes
    mongodb_store::MessageStoreProxy table_shape(*nh, "table_shapes");
    std::vector< boost::shared_ptr<strands_perception_msgs::Table> > result_tables;
    table_shape.query<strands_perception_msgs::Table>(result_tables);

    //set or load checked_num
    std::vector< boost::shared_ptr<std_msgs::Int32> > checked_amount;
    table_shape.queryNamed<std_msgs::Int32>("checked_num", checked_amount);
    if(checked_amount.size() == 0){
        std_msgs::Int32 tmp;
        tmp.data = checked_num;
        table_shape.insertNamed("checked_num",tmp);
    }
    else{
        //load record form mongodb
        checked_num = checked_amount[0]->data;
    }
    ROS_INFO("Found %lu table shapes, already checked %d", result_tables.size(), checked_num);
    //extract points from msg

    pcl_cloud::Ptr new_cloud(new pcl_cloud());
    for(int j=0;j<result_tables.size();j++){

        for(int i=0;i<(*result_tables[j]).tabletop.points.size();i++){
            Point tt;
            tt.x = (*result_tables[j]).tabletop.points[i].x;
            tt.y = (*result_tables[j]).tabletop.points[i].y;
            tt.z = (*result_tables[j]).tabletop.points[i].z;
            new_cloud->push_back(tt);
        }

    }

    return true;

}

//extract table clouds, convex hull, and convert to msg stored in DB
bool extract_table_msg(pcl_cloud::Ptr cloud_in)
{
    pcl_cloud::Ptr cloud1(new pcl_cloud());
    pcl_cloud::Ptr cloud_out(new pcl_cloud());

    //filter out range that may not be a table plane
    pcl::PassThrough<Point> pass;
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.45,1.2);
    pass.setInputCloud(cloud_in);
    pass.filter(*cloud1);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<Point> seg;

    //pcl::Normal nor;
    //nor.normal_x=0.0;
    //nor.normal_y=0.0;
    //nor.normal_z=1.0;
    //pcl::SampleConsensusModelNormalPlane<Point, pcl::Normal> normal_sac();

    seg.setOptimizeCoefficients (true);
    //plane model
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

    if(Debug){
        std::cout<<nor.normal_x<<std::endl;
        std::cout<<nor.normal_y<<std::endl;
        std::cout<<nor.normal_z<<std::endl;
    }

    if(nor.normal_x>0.1||nor.normal_y>0.1||nor.normal_z>0.1){
        ROS_INFO("Normal direction exceed threshold, skip.");
        return false;
    }

    pcl_cloud::Ptr cloud_hull(new pcl_cloud());
    extract_convex(cloud1,inliers,coefficients,cloud_hull);
    ROS_INFO("Convex cloud has been extracted contains %lu points", (*cloud_hull).points.size());
    if(Debug){
        std::string name="/home/parallels/debug/.pcd";
        std::stringstream ss;
        ss<< "convex";
        std::string str = ss.str();
        name.insert(name.length()-4, str);
        std::cout<<name<<std::endl;
        pcl::io::savePCDFileASCII(name, *cloud_hull);
    }

    //construct a table msg
    strands_perception_msgs::Table table_msg;
    table_msg.pose.pose.orientation.w = 1.0;
    table_msg.header.frame_id = "/map";
    table_msg.header.stamp = ros::Time();
    for(int i=0;i<(*cloud_hull).points.size();i++){
        geometry_msgs::Point32 pp;
        pp.x = (*cloud_hull).at(i).x;
        pp.y = (*cloud_hull).at(i).y;
        pp.z = (*cloud_hull).at(i).z;
        table_msg.tabletop.points.push_back(pp);
    }

    //insert to mongodb
    mongodb_store::MessageStoreProxy table_shape(*nh, "table_shapes");
    table_shape.insert(table_msg);
    ROS_INFO("Insert table shape to collection.");

    return true;
}

//extract whole table clouds, stored in DB
bool extract_table(pcl_cloud::Ptr cloud)
{

    pcl_cloud::Ptr cloud1(new pcl_cloud());
    pcl_cloud::Ptr cloud_out(new pcl_cloud());

    //filter out range that may not be a table plane
    pcl::PassThrough<Point> pass;
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.45,1.2);
    pass.setInputCloud(cloud);
    pass.filter(*cloud1);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    //pcl::Normal nor;
    //nor.normal_x=0.0;
    //nor.normal_y=0.0;
    //nor.normal_z=1.0;
    //pcl::SampleConsensusModelNormalPlane<Point, pcl::Normal> normal_sac();

    seg.setOptimizeCoefficients (true);
    //plane model
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

    if(nor.normal_x>0.1||nor.normal_y>0.1||nor.normal_z>0.1){
        ROS_INFO("Normal direction exceed threshold, skip.");
        return false;
    }

    //conversion
    sensor_msgs::PointCloud2 table_cloud;
    pcl::toROSMsg(*cloud_out, table_cloud);

    //store to mongodb
    mongodb_store::MessageStoreProxy messageStore(*nh,"whole_table_clouds");
    messageStore.insert(table_cloud);

     pcl_cloud::Ptr cloud_hull(new pcl_cloud());
    extract_convex(cloud1,inliers,coefficients,cloud_hull);
    ROS_INFO("Convex cloud has been extracted contains %lu points", (*cloud_hull).points.size());
    if(Debug){
        std::string name="/home/parallels/debug/.pcd";
        std::stringstream ss;
        ss<< "convex";
        std::string str = ss.str();
        name.insert(name.length()-4, str);
        std::cout<<name<<std::endl;
        pcl::io::savePCDFileASCII(name, *cloud_hull);
    }

    //construct a table msg
    strands_perception_msgs::Table table_msg;
    table_msg.pose.pose.orientation.w = 1.0;
    table_msg.header.frame_id = "/map";
    table_msg.header.stamp = ros::Time();
    for(int i=0;i<(*cloud_hull).points.size();i++){
        geometry_msgs::Point32 pp;
        pp.x = (*cloud_hull).at(i).x;
        pp.y = (*cloud_hull).at(i).y;
        pp.z = (*cloud_hull).at(i).z;
        table_msg.tabletop.points.push_back(pp);
    }

    //insert to mongodb
    mongodb_store::MessageStoreProxy table_shape(*nh, "whole_table_shapes");
    table_shape.insert(table_msg);
    ROS_INFO("Insert whole table shape to collection.");

}

//store icp cloud -> table cloud plane to DB
bool extract(table_detection::db_table_clouds::Request &req, table_detection::db_table_clouds::Response &res)
{

    pcl_cloud::Ptr loaded_cloud(new pcl_cloud());
    pcl::fromROSMsg(req.cloud, *loaded_cloud);
    bool rc = extract_table(loaded_cloud);

    //extract convex hull for whole table

    return true;
}

//extract table and construct a strands_perception_msg, store it in DB call this service and pass in a point cloud
bool extract2(table_detection::db_table::Request &req, table_detection::db_table::Response &res)
{
    //copy cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(req.cloud,*cloud_in);
    //pass it to table extraction
    bool rc = extract_table_msg(cloud_in);

    return rc;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "db_cloud_extraction");
    nh.reset(new ros::NodeHandle);
    ros::NodeHandle pn("~");

    ros::ServiceServer table_srv = nh->advertiseService("db_table_clouds", extract);
    ros::ServiceServer table_srv2 = nh->advertiseService("db_table", extract2);

    ros::spin();

}