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

#define Debug true
typedef pcl::PointXYZ  Point;
typedef pcl::PointCloud<Point> pcl_cloud;
ros::NodeHandlePtr nh;
int checked_num=0;
int table_knn;

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

//check two convex hull if have overlap
bool overlap_check()
{

}

//cross product calculate size of polygen
double table_size(strands_perception_msgs::Table& table)
{

}

//loading msg form collection and merge tables that may linked together
bool merge_table_msg()
{
    //load table shapes
    mongodb_store::MessageStoreProxy table_merge(*nh, "table_merge");
    std::vector< boost::shared_ptr<strands_perception_msgs::Table> > result_tables;
    table_merge.query<strands_perception_msgs::Table>(result_tables);

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

//check if two convex hull have overlap
bool overlap(){

}

bool table_centre_group()
{
    //load table centres and visualization
    mongodb_store::MessageStoreProxy table_centre(*nh, "table_centre");
    std::vector<boost::shared_ptr<geometry_msgs::Point32> >result_tables;
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
    for(int i=checked_num;i<result_tables.size();i++){
        points_arr.points.push_back(*result_tables[i]);
    }
    table_centre_group.insert(points_arr);

    return true;
    /*
    //TODO:(be careful)indices has at least the size of knn, if no neighbour just set the index 0
    if(indices[1]!=0){
        //delete that neighbour
        if(!merge_able){
            //indices[1]
        }
    }
     */
}

bool merge(table_detection::db_merge::Request& req, table_detection::db_merge::Response& res)
{
    //check nearby table shape
    //
    return true;
}

bool group_centre(table_detection::db_table_centre::Request& req, table_detection::db_table_centre::Response& res)
{
    //group all unchecked table centre to a new collection in one data(Polygen a bunch of points).
    bool rc = table_centre_group();

    return rc;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "db_cloud_extraction");
    nh.reset(new ros::NodeHandle);
    ros::NodeHandle pn("~");

    //can only use 1 neighbour
    pn.param<int>("table_knn", table_knn, 1);
    ros::ServiceServer table_srv = nh->advertiseService("db_merge", merge);
    //store one round table centre
    ros::ServiceServer table_centre_srv = nh->advertiseService("db_table_centre", group_centre);

    table_kdtree();

    ros::spin();

}