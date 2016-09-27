//
//table points , structure, properties
//also contains some db operations
//

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <table_detection/Table.h>
#include <table_detection/table_neighbour_arr.h>
#include <mongodb_store/message_store.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/core/core.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/surface/convex_hull.h>
#include <table_detection/Table.h>

#ifndef TABLE_MAPPING_TABLE_TOOL_H
#define TABLE_MAPPING_TABLE_TOOL_H

#define Debug true

template <class Point>
class Table{

public:

    typedef Point point_type;
    typedef pcl::PointCloud<point_type> cloud_type;
    typedef typename cloud_type::Ptr cloud_ptr;
    typedef typename cloud_type::ConstPtr cloud_const_ptr;

    Table();

    //if you want to use mongodb options
    Table(ros::NodeHandlePtr nh_in);


    /*
     *
     */
    void table_normal();
    /*
     * Given index of table merge them using convex hull
     */
    void table_merge(std::vector<std::vector<int> > merge_index);
    /*
     * A point in polygen check
     * run after one round observation
     * return ture if merge able
     */
    void overlap_detect(std::string collection, std::vector<std::vector<int> > &overlap_index);
    /*
     * Given collection that contains table centre group(table centre generated by each round)
     * try to merge table centres that are close enough
     * */
    void merge_table_centre(std::string collection);

    /* construct a kdtree to store table centre(which is calculate during each scan), (load all table cloud from db)
     * Given db collection that contains sensor_msgs::PointCloud2
     * calculate all the centre in given collection, and put in a given pcl kdtree
     */
    void dbtable_kdtree(std::string collection, pcl::KdTreeFLANN<point_type> &kdtree);

    /*
     * Instead of table centre using all table clouds extracted
     */
    void dbtable_cloud_kdtree(std::string collection, pcl::KdTreeFLANN<point_type> &kdtree, int begin, int end);

    /*
     * calculate table centre for a single scan
     * store_point and collection is enabled by default,
     * return cv::Point3f, centre point.
     */
    point_type table_cloud_centre(sensor_msgs::PointCloud2 msg,
                                   bool store_point=true,
                                   std::string collection="table_centre");
    void dbtable_cloud_centre(std::string collection, std::vector<point_type>& table_centre_index);

private:
    //need init by ros node
    ros::NodeHandlePtr nh;

};
#endif //TABLE_MAPPING_TABLE_TOOL_H


