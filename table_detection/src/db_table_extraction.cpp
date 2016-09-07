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
#include <table_detection/Table.h>
#include <std_msgs/Int32.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "table_detection/table_tool.hpp"


#define Debug true
#define PI 3.1415926
//typedef pcl::PointXYZRGB  Point;
typedef pcl::PointXYZ  Point;
typedef pcl::PointCloud<Point> pcl_cloud;
ros::NodeHandlePtr nh;
int checked_num=0;

float normal_angle;
float search_radius;
int neighbour_required;
int statistical_knn;
float std_dev_dist;

void extract_convex(pcl_cloud::Ptr cloud_in,
                    pcl::PointIndices::Ptr inliers,
                    pcl::ModelCoefficients::Ptr coefficients,
                    pcl_cloud::Ptr cloud_hull,
                    pcl_cloud::Ptr cloud_cave)

{
    pcl_cloud::Ptr projected(new pcl_cloud());

    pcl::ProjectInliers<Point> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setIndices (inliers);
    proj.setInputCloud (cloud_in);
    proj.setModelCoefficients (coefficients);
    proj.filter (*projected);

    pcl::ConvexHull<Point> chull;
    chull.setInputCloud (projected);
    chull.reconstruct (*cloud_hull);

    pcl::ConcaveHull<Point> cavehull;
    cavehull.setAlpha(0.1);
    cavehull.setInputCloud (projected);
    cavehull.reconstruct (*cloud_cave);
}

//loading msg form collection and merge tables that may linked together
bool merge_table_msg()
{
    //load table shapes
    mongodb_store::MessageStoreProxy table_shape(*nh, "table_shapes");
    std::vector< boost::shared_ptr<table_detection::Table> > result_tables;
    table_shape.query<table_detection::Table>(result_tables);

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

//remove points that are sparsely distributed
void outlier_filter_statistical(pcl_cloud::Ptr cloud_in, pcl_cloud::Ptr cloud_out)
{
    pcl::StatisticalOutlierRemoval<Point> sor;
    sor.setInputCloud (cloud_in);
    sor.setMeanK (statistical_knn);
    sor.setStddevMulThresh (std_dev_dist);
    sor.filter (*cloud_out);
    ROS_INFO("Statistical Remover, Before: %lu After: %lu",cloud_in->size(),cloud_out->size());
}

//remove points if can't find given number of neighbour in given radius
void outlier_filter_radius(pcl_cloud::Ptr cloud_in, pcl_cloud::Ptr cloud_out)
{
    pcl::RadiusOutlierRemoval<Point> outrem;
    outrem.setInputCloud(cloud_in);
    outrem.setRadiusSearch(search_radius);
    outrem.setMinNeighborsInRadius (neighbour_required);
    outrem.filter (*cloud_out);
}

void outlier_filter_table_centre(pcl_cloud::Ptr cloud_in, pcl_cloud::Ptr cloud_out)
{
    //computer plane center
    //check points distributions
    //remove points far away from centre
}

void convex_size(){

}

//extract table clouds, convex hull, and convert to msg stored in DB
bool extract_table_msg(pcl_cloud::Ptr cloud_in, bool is_whole, bool store_cloud=false, bool store_convex=false)
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

    //filter out noise
    pcl_cloud::Ptr cloud_nonoise(new pcl_cloud());
    outlier_filter_radius(cloud_out,cloud_nonoise);

    //normal estimation and filter table plane
    pcl::NormalEstimationOMP<Point, pcl::Normal> ne;
    ne.setInputCloud (cloud_nonoise);
    pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point> ());
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
    //normalize instead of average
    float normal_length = sqrt(nor.normal_x*nor.normal_x+nor.normal_y*nor.normal_y+nor.normal_z*nor.normal_z);
    nor.normal_x = nor.normal_x/normal_length;
    nor.normal_y = nor.normal_y/normal_length;
    nor.normal_z = nor.normal_z/normal_length;

    if(Debug){
        std::cout<<nor.normal_x<<std::endl;
        std::cout<<nor.normal_y<<std::endl;
        std::cout<<nor.normal_z<<std::endl;
    }

    //the default viewpoint is (0,0,0),thus using global cloud some plane and viewpoint are in a same plane,
    //which may not helpful to flip normal direction
    pcl::Normal standard_normal;
    standard_normal.normal_x = 0.0;
    standard_normal.normal_y = 0.0;
    standard_normal.normal_z = 1.0;
    float cosin_angle = (standard_normal.normal_z*nor.normal_z);
    float tmp_angle = acos(cosin_angle) * (180.0/PI);

    //if(tmp_angle>90.0?(tmp_angle-90.0)<normal_angle:tmp_angle<normal_angle){
    if(tmp_angle-90.0>0.0){
        if((180.0 - tmp_angle)<normal_angle){
            ROS_INFO("Normal direction meet requirement %f, angle calculated is: 180.0-%f=%f. ", normal_angle, tmp_angle, 180.0-tmp_angle);
        }
        else{
            ROS_INFO("Normal direction exceed threshold %f, angle calculated is: 180.0-%f=%f. skip.", normal_angle, tmp_angle, 180.0-tmp_angle);
            return false;
        }
    }
    else{
        if(tmp_angle<normal_angle) {
            ROS_INFO("Normal direction meet requirement angle %f, angle calculated is %f", normal_angle, tmp_angle);
        }
        else{
            ROS_INFO("Normal direction exceed threshold %f, angle calculated is: %f. skip.", normal_angle, tmp_angle);
            return false;
        }
    }

    pcl_cloud::Ptr cloud_hull(new pcl_cloud());
    pcl_cloud::Ptr cloud_cave(new pcl_cloud());
    extract_convex(cloud1,inliers,coefficients,cloud_hull,cloud_cave);
    //reject some size convex

    //store the plane that pass the table filter
    if(store_cloud) {
        ROS_INFO("Storing table cloud (noise & filted)...");
        if (is_whole) {
            mongodb_store::MessageStoreProxy table_whole_cloud_noise(*nh, "whole_table_clouds_noise");
            mongodb_store::MessageStoreProxy table_whole_cloud(*nh, "whole_table_clouds");

            //conversion
            sensor_msgs::PointCloud2 whole_table_cloud_noise;
            pcl::toROSMsg(*cloud_out, whole_table_cloud_noise);
            table_whole_cloud_noise.insert(whole_table_cloud_noise);

            sensor_msgs::PointCloud2 whole_table_cloud;
            pcl::toROSMsg(*cloud_nonoise, whole_table_cloud);
            table_whole_cloud.insert(whole_table_cloud);

        }
        else {
            mongodb_store::MessageStoreProxy dbtable_cloud_noise(*nh, "table_clouds_noise");
            mongodb_store::MessageStoreProxy dbtable_cloud(*nh, "table_clouds");
            //conversion
            sensor_msgs::PointCloud2 table_cloud_noise;
            pcl::toROSMsg(*cloud_out, table_cloud_noise);
            dbtable_cloud_noise.insert(table_cloud_noise);

            sensor_msgs::PointCloud2 table_cloud;
            pcl::toROSMsg(*cloud_nonoise, table_cloud);
            dbtable_cloud.insert(table_cloud);

            //store table centre for each scan
            ROS_INFO("Storing table centre...");
            Table<Point> tb(nh);
            tb.table_cloud_centre(table_cloud);
        }
    }


    ROS_INFO("Convex cloud has been extracted contains %lu points", (*cloud_hull).points.size());

    if(store_convex){
        ROS_INFO("Storing convex cloud...");
        if(is_whole){
            mongodb_store::MessageStoreProxy dbtable_whole_convex_cloud(*nh, "whole_table_convex");
            mongodb_store::MessageStoreProxy dbtable_whole_concave_cloud(*nh, "whole_table_concave");
            //conversion
            sensor_msgs::PointCloud2 whole_table_convex;
            pcl::toROSMsg(*cloud_hull, whole_table_convex);
            dbtable_whole_convex_cloud.insert(whole_table_convex);

            sensor_msgs::PointCloud2 whole_table_concave;
            pcl::toROSMsg(*cloud_cave, whole_table_concave);
            dbtable_whole_concave_cloud.insert(whole_table_concave);
        }
        else{
            mongodb_store::MessageStoreProxy dbtable_convex_cloud(*nh, "table_convex");
            mongodb_store::MessageStoreProxy dbtable_concave_cloud(*nh, "table_concave");
            //conversion
            sensor_msgs::PointCloud2 table_convex;
            pcl::toROSMsg(*cloud_hull, table_convex);
            dbtable_convex_cloud.insert(table_convex);

            sensor_msgs::PointCloud2 table_concave;
            pcl::toROSMsg(*cloud_cave, table_concave);
            dbtable_concave_cloud.insert(table_concave);
        }
    }

    //construct a table msg
    table_detection::Table table_msg;
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
    if(is_whole){
        mongodb_store::MessageStoreProxy whole_table_shape(*nh, "whole_table_shapes");
        whole_table_shape.insert(table_msg);
        ROS_INFO("Insert whole table shape to collection.");
    }
    else{
        mongodb_store::MessageStoreProxy table_shape(*nh, "table_shapes");
        table_shape.insert(table_msg);
        ROS_INFO("Insert table shape to collection.");
    }
    return true;
}

//store icp cloud -> table cloud plane to DB
bool extract(table_detection::db_table_clouds::Request &req, table_detection::db_table_clouds::Response &res)
{

    pcl_cloud::Ptr loaded_cloud(new pcl_cloud());
    pcl::fromROSMsg(req.cloud, *loaded_cloud);
    bool rc = extract_table_msg(loaded_cloud, true, true, true);

    return rc;
}

//extract table and construct a table_detection table msg, store it in DB call this service and pass in a point cloud
bool extract2(table_detection::db_table::Request &req, table_detection::db_table::Response &res)
{
    //copy cloud
    pcl::PointCloud<Point>::Ptr cloud_in(new pcl::PointCloud<Point>);
    pcl::fromROSMsg(req.cloud,*cloud_in);
    //pass it to table extraction
    bool rc = extract_table_msg(cloud_in, false, true, true);

    return rc;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "db_cloud_extraction");
    nh.reset(new ros::NodeHandle);
    ros::NodeHandle pn("~");

    //plane filter
    pn.param<float>("normal_angle", normal_angle, 15.0);
    //filter1
    pn.param<float>("search_radius", search_radius, 0.8);
    pn.param<int>("neighbour_required", neighbour_required, 20);
    //filter2
    pn.param<int>("statistical_knn", statistical_knn, 50);
    pn.param<float>("std_dev_dist", std_dev_dist, 1.0);

    ros::ServiceServer table_srv = nh->advertiseService("db_table_clouds", extract);
    ros::ServiceServer table_srv2 = nh->advertiseService("db_table", extract2);

    ros::spin();

}