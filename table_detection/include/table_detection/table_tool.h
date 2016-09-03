//
//table points , structure, properties
//also contains some db operations
//

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <strands_perception_msgs/Table.h>
#include <mongodb_store/message_store.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/core/core.hpp>
#include <pcl/kdtree/kdtree_flann.h>

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


    /* construct a kdtree to store table centre(which is calculate during each scan), (load all table cloud from db)
     * Given db collection that contains sensor_msgs::PointCloud2
     * calculate all the centre in given collection, and put in a given pcl kdtree
     */
    void dbtable_kdtree(std::string collection, pcl::KdTreeFLANN<point_type> &kdtree);

    /*
     * Instead of table centre using all table clouds extracted
     */
    void dbtable_cloud_kdtree(std::string collection, pcl::KdTreeFLANN<point_type> &kdtree);

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


