#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point32.h>
#include <mongodb_store/message_store.h>
#include <pcl_ros/point_cloud.h>

/*
 * Function as a plugin api for visualizing data
 *2D and 3D viz support
 * 1. need ros handler Ptr, call pub api
 * */
#ifndef VIZ_POINTS_H
#define VIZ_POINTS_H

class VIZ_Points{

public:
    typedef pcl::PointXYZ point_type;
    typedef pcl::PointCloud<point_type> cloud_type;
    typedef typename cloud_type::Ptr cloud_ptr;
    typedef typename cloud_type::ConstPtr cloud_const_ptr;

    //init with ros node handler, and data that you want to view in rviz
    VIZ_Points(ros::NodeHandlePtr nhp, std::string topic);

    VIZ_Points(ros::NodeHandlePtr nhp);


    //pub viz marker if given points array
    void pub_points(std::vector<boost::shared_ptr<geometry_msgs::Point32> >& points_arr);
    //pub circle marker if given circle centre array
    void pub_circle(std::vector<boost::shared_ptr<geometry_msgs::Point32> >& centre_arr);
    //pub db sensor_msgs::Pointcloud2 msg if given index
    void dbpub_points(std::string collection, std::vector<int> indices);

private:
    ros::Publisher viz_pub;
    ros::NodeHandlePtr viz_n;
    visualization_msgs::MarkerArray mark_arr;
    std::string namespace_of;

};

#endif //VIZ_POINTS_H
