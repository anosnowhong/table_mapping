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
#include <pcl/features/normal_3d.h>
#include <pcl/surface/concave_hull.h>
#include <std_msgs/Int32.h>

#include <table_detection/db_table_clouds.h>
#include <table_detection/db_table.h>
#include <table_detection/db_merge.h>
#include <table_detection/db_table_centre.h>
#include "table_detection/viz_points.h"
#include "table_detection/table_tool.h"
#include <table_detection/table_neighbour.h>
#include <table_detection/table_neighbour_arr.h>

#define Debug true
typedef pcl::PointXYZ  Point;
typedef pcl::PointCloud<Point> pcl_cloud;
ros::NodeHandlePtr nh;
int checked_num=0;
int table_knn;
float bad_observation_remove_radius;

//cross product calculate size of polygen
double table_size(table_detection::Table& table)
{

}

//loading msg form collection and merge tables that may linked together
bool merge_table_msg()
{
    //load table shapes
    mongodb_store::MessageStoreProxy table_merge(*nh, "table_merge");
    std::vector< boost::shared_ptr<table_detection::Table> > result_tables;
    table_merge.query<table_detection::Table>(result_tables);

    //set or load checked_num
    std::vector< boost::shared_ptr<std_msgs::Int32> > checked_amount;
    table_merge.queryNamed<std_msgs::Int32>("checked_num", checked_amount);
    if(checked_amount.size() == 0){
        std_msgs::Int32 tmp;
        tmp.data = checked_num;
        table_merge.insertNamed("checked_num",tmp);
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

bool table_centre_group()
{
    //load table centres and visualization
    mongodb_store::MessageStoreProxy table_centre(*nh, "table_centre");
    std::vector<boost::shared_ptr<geometry_msgs::Point32> > result_tables;
    table_centre.query<geometry_msgs::Point32>(result_tables);

    if(Debug) {
        ROS_INFO("table centre number: %lu", result_tables.size());
        //publish and view once
        VIZ_Points vv(nh,"/table_centers");
        vv.pub_points(result_tables);
    }

    //put one round table centre in a group
    mongodb_store::MessageStoreProxy table_centre_group(*nh, "table_centre_group");

    //set or load checked_num
    std::vector< boost::shared_ptr<std_msgs::Int32> > checked_amount;
    table_centre_group.queryNamed<std_msgs::Int32>("checked_num", checked_amount);
    if(checked_amount.size() == 0){
        std_msgs::Int32 tmp;
        tmp.data = checked_num;
        table_centre_group.insertNamed("checked_num",tmp);
    }
    else{
        //load record form mongodb
        checked_num = checked_amount[0]->data;
    }
    ROS_INFO("Found %lu table centres, already checked %d", result_tables.size(), checked_num);

    //start inserting
    geometry_msgs::Polygon points_arr;
    for(;checked_num<result_tables.size();checked_num++){
        points_arr.points.push_back(*result_tables[checked_num]);
    }
    table_centre_group.insert(points_arr);

    //update check_num in mongodb
    std_msgs::Int32 tmp;
    tmp.data = checked_num;
    table_centre_group.updateNamed("checked_num",tmp);

    return true;
    /*
    //TODO:(be careful)indices has at least the size of knn, if no neighbour just set the index 0
    if(indces[1]!=0){
        //delete that neighbour
        if(!merge_able){
            //indices[1]
        }
    }
     */

}

//distance based filter, distance measured by point from table centre
void outlier_remove(pcl_cloud::Ptr cloud_in, pcl_cloud::Ptr cloud_out, Point pp){
    float threshold = 1.2;

    float dis,xx,yy, xc,yc;
    xc=pp.x;
    yc=pp.y;

    for(int i=0;i<cloud_in->points.size();i++){
        xx = cloud_in->points.at(i).x;
        yy = cloud_in->points.at(i).y;

        dis = sqrt( pow((xx-xc),2)+pow((yy-yc),2) );
        if(dis<threshold){
            cloud_out->push_back(cloud_in->points.at(i));
        }
    }
}
//need at least 2 round to analyse data.
void table_increment_nei()
{
    //loading all grouped table centres
    mongodb_store::MessageStoreProxy table_centre_group(*nh, "table_centre_group");
    std::vector<boost::shared_ptr<geometry_msgs::Polygon> > result_tables;
    table_centre_group.query<geometry_msgs::Polygon>(result_tables,mongo::BSONObj(),mongo::BSONObj(),BSON("_meta.inserted_at"<<1));
    mongodb_store::MessageStoreProxy table_convex(*nh, "table_convex");
    std::vector<boost::shared_ptr<sensor_msgs::PointCloud2> > convex_info;
    table_convex.query<sensor_msgs::PointCloud2>(convex_info,mongo::BSONObj(),mongo::BSONObj(),BSON("_meta.inserted_at"<<1));
    mongodb_store::MessageStoreProxy table_concave(*nh, "table_concave");
    std::vector<boost::shared_ptr<sensor_msgs::PointCloud2> > concave_info;
    table_concave.query<sensor_msgs::PointCloud2>(concave_info,mongo::BSONObj(),mongo::BSONObj(),BSON("_meta.inserted_at"<<1));

    ROS_INFO("Start calculating increment...");
    //all table cloud kd tree(single scan)
    Table<Point> checker(nh);
    pcl::KdTreeFLANN<Point> kdtree;

    //the last one is newly inserted, get how many table has been extracted
    std::vector< boost::shared_ptr<std_msgs::Int32> > checked_amount;
    table_centre_group.queryNamed<std_msgs::Int32>("checked_num", checked_amount);
    int t_end = checked_amount[0]->data;
    int t_begin = t_end - result_tables[result_tables.size()-1]->points.size();
    //construct kd tree for one round table cloud
    checker.dbtable_cloud_kdtree("table_clouds", kdtree, t_begin, t_end);

    /*
     *  use point cloud from one round
     *  get neighbour query result for all table centre
     */
    std::vector<int> point_index;
    std::vector<float> point_distance;
    table_detection::table_neighbour_arr neighbour_arr;
    table_detection::table_neighbour neighbour;

    //different round data
    for(int i=0;i<result_tables.size();i++){
        for(int j=0;j<result_tables[i]->points.size();j++){
            Point query_p;
            //query pcl point
            query_p.x = result_tables[i]->points.at(j).x;
            query_p.y = result_tables[i]->points.at(j).y;
            query_p.z = result_tables[i]->points.at(j).z;
            //copy to ros point32 as well
            int nei = kdtree.radiusSearch(query_p,bad_observation_remove_radius,point_index,point_distance);
            neighbour.table_centre = result_tables[i]->points.at(j);
            neighbour.neighbour.data = nei;

            //Remove outliers
            Point pp;
            pp.x = result_tables[i]->points.at(j).x;
            pp.y = result_tables[i]->points.at(j).y;
            pp.z = result_tables[i]->points.at(j).z;
            pcl_cloud::Ptr cloudin(new pcl_cloud());
            pcl_cloud::Ptr cloudout(new pcl_cloud());
            pcl::fromROSMsg(*convex_info[j],*cloudin);
            outlier_remove(cloudin,cloudout,pp);
            sensor_msgs::PointCloud2 convex_in;
            pcl::toROSMsg(*cloudout,convex_in);

            neighbour.convex_cloud = convex_in;

            neighbour.concave_cloud = *concave_info[j];
            neighbour_arr.neighbour_arr.push_back(neighbour);
        }
    }
    //store the neighbour neighbour
    mongodb_store::MessageStoreProxy table_centre_increment(*nh, "table_centre_increment");
    table_centre_increment.insert(neighbour_arr);
    ROS_INFO("Done!");
}

bool merge(table_detection::db_merge::Request& req, table_detection::db_merge::Response& res)
{
    Table<Point> tb(nh);
    std::vector<std::vector<int> > merge_info;
    tb.overlap_detect("table_centre_increment", merge_info);

    /*
    if(Debug){
        VIZ_Points vv(nh);
        vv.dbpub_points("table_convex",merge_info[2]);
    }
     */

    tb.table_merge(merge_info);

    return true;
}

bool group_centre(table_detection::db_table_centre::Request& req, table_detection::db_table_centre::Response& res)
{
    //group all unchecked table centre to a new collection in one data(Polygen a bunch of points).
    bool rc = table_centre_group();
    //calculate increment and store
    table_increment_nei();
    //store round info
    //mongodb_store::MessageStoreProxy round_info(*nh, "table_clouds");
    //round_info.insertNamed("")

    return rc;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "db_cloud_extraction");
    nh.reset(new ros::NodeHandle);
    ros::NodeHandle pn("~");

    //can only use 1 neighbour
    pn.param<int>("table_knn", table_knn, 1);
    pn.param<float>("bad_observation_remove_radius", bad_observation_remove_radius, 0.2);
    ros::ServiceServer table_srv = nh->advertiseService("db_merge", merge);
    //store one round table centre
    ros::ServiceServer table_centre_srv = nh->advertiseService("db_table_centre", group_centre);

    ros::spin();

}
