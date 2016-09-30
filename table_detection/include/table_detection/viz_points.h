#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point32.h>
#include <mongodb_store/message_store.h>
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/Polygon.h>

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

    //init a polygen publisher
    VIZ_Points(ros::NodeHandlePtr nhp, std::string topic, int polygon);

    VIZ_Points(ros::NodeHandlePtr nhp);


    //void pub_polygon(std::vector<boost::shared_ptr<geometry_msgs::Polygon> >& polygon);

    //publish polygon as polygon
    void pub_polygon(std::vector<boost::shared_ptr<geometry_msgs::Polygon> >& polygons, int index, std::vector<float> color);
    //pub viz marker if given polygen similar to pub_points but the input msg is different
    void pub_polygonASpoints(std::vector<boost::shared_ptr<geometry_msgs::Polygon> >& polygons, int index, std::vector<float> color);
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

VIZ_Points::VIZ_Points(ros::NodeHandlePtr nhp){

    viz_n = nhp;
    viz_pub = viz_n->advertise<sensor_msgs::PointCloud2>("/global_cloud", 1);
    ros::Duration(1).sleep();
    ROS_INFO("=====ss");
}

VIZ_Points::VIZ_Points(ros::NodeHandlePtr nhp, std::string topic) {

    //init publisher and get namespace form node
    viz_pub = nhp->advertise<visualization_msgs::MarkerArray>(topic, 1);
    namespace_of = nhp->getNamespace();
    //give time to register the publisher
    ros::Duration(1).sleep();

}

VIZ_Points::VIZ_Points(ros::NodeHandlePtr nhp, std::string topic,int polygon){
    viz_pub = nhp->advertise<geometry_msgs::Polygon>(topic, 1);
    namespace_of = nhp->getNamespace();
    //give time to register the publisher
    ros::Duration(1).sleep();
}

void VIZ_Points::pub_points(std::vector<boost::shared_ptr<geometry_msgs::Point32> >& points_arr) {

    mark_arr.markers.resize(points_arr.size());
    ROS_INFO("VIZ POINTS: marker array size %lu", points_arr.size());

    /*
    //wait untill there is at least one subscribe
    while (viz_pub.getNumSubscribers() == 0){
        ros::Duration(0.1).sleep();
    }
     */

    for(int i=0;i<points_arr.size();i++)
    {
        mark_arr.markers[i].header.frame_id="/map";
        mark_arr.markers[i].header.stamp = ros::Time();
        mark_arr.markers[i].ns = namespace_of;
        mark_arr.markers[i].id = i;
        mark_arr.markers[i].type=visualization_msgs::Marker::CYLINDER;
        mark_arr.markers[i].action= visualization_msgs::Marker::ADD;

        mark_arr.markers[i].pose.position.x = points_arr[i]->x;
        mark_arr.markers[i].pose.position.y = points_arr[i]->y;
        mark_arr.markers[i].pose.position.z = 0;

        mark_arr.markers[i].pose.orientation.x = 0;
        mark_arr.markers[i].pose.orientation.y = 0;
        mark_arr.markers[i].pose.orientation.z = 0;
        mark_arr.markers[i].pose.orientation.w = 1;

        mark_arr.markers[i].scale.x = 0.1;
        mark_arr.markers[i].scale.y = 0.1;
        mark_arr.markers[i].scale.z = 0.1;
        mark_arr.markers[i].color.a = 1.0;
        mark_arr.markers[i].color.r = 0.0;
        mark_arr.markers[i].color.g = 1.0;
        mark_arr.markers[i].color.b = 0.0;
    }
    ROS_INFO("VIZ POINTS: marker array has been published!");
    viz_pub.publish(mark_arr);

}

void VIZ_Points::pub_polygon(std::vector<boost::shared_ptr<geometry_msgs::Polygon> >& polygons, int index, std::vector<float> color){

}

void VIZ_Points::pub_polygonASpoints(std::vector<boost::shared_ptr<geometry_msgs::Polygon> > &polygons, int index,
                                     std::vector<float> color){
    mark_arr.markers.resize(polygons[index]->points.size());
    ROS_INFO("VIZ POINTS: marker array size %lu", polygons[index]->points.size());

    for(int i=0;i<polygons[index]->points.size();i++)
    {
        mark_arr.markers[i].header.frame_id="/map";
        mark_arr.markers[i].header.stamp = ros::Time();
        mark_arr.markers[i].ns = namespace_of;
        mark_arr.markers[i].id = i;
        mark_arr.markers[i].type=visualization_msgs::Marker::CYLINDER;
        mark_arr.markers[i].action= visualization_msgs::Marker::ADD;

        mark_arr.markers[i].pose.position.x = polygons[index]->points.at(i).x;
        mark_arr.markers[i].pose.position.y = polygons[index]->points.at(i).y;
        mark_arr.markers[i].pose.position.z = 0;

        mark_arr.markers[i].pose.orientation.x = 0;
        mark_arr.markers[i].pose.orientation.y = 0;
        mark_arr.markers[i].pose.orientation.z = 0;
        mark_arr.markers[i].pose.orientation.w = 1;

        mark_arr.markers[i].scale.x = 0.1;
        mark_arr.markers[i].scale.y = 0.1;
        mark_arr.markers[i].scale.z = 0.1;
        mark_arr.markers[i].color.a = 1.0;
        mark_arr.markers[i].color.r = color[0];
        mark_arr.markers[i].color.g = color[1];
        mark_arr.markers[i].color.b = color[2];
    }
    ROS_INFO("VIZ POINTS: marker array has been published!");
    viz_pub.publish(mark_arr);
}

void VIZ_Points::pub_circle(std::vector<boost::shared_ptr<geometry_msgs::Point32> >& centre_arr){
    //define radius
    //write msg
    //publist
}

void VIZ_Points::dbpub_points(std::string collection, std::vector<int> indices) {

    mongodb_store::MessageStoreProxy messageStore(*viz_n, collection);

    std::vector<boost::shared_ptr<sensor_msgs::PointCloud2> > result_pc2;

    messageStore.query<sensor_msgs::PointCloud2>(result_pc2);

    ROS_INFO("Loading %lu Point Clouds!", indices.size());

    cloud_ptr cloud_sum(new cloud_type());
    cloud_ptr cloud_store(new cloud_type());

    for (int i = 0; i < indices.size(); i++) {
        pcl::fromROSMsg(*result_pc2[indices[i]], *cloud_store);
        *cloud_sum += *cloud_store;
    }

    ROS_INFO("Publishing...");
    sensor_msgs::PointCloud2 pub_cloud;
    pcl::toROSMsg(*cloud_sum,pub_cloud);

    pub_cloud.header = result_pc2[0]->header;
    pub_cloud.header.stamp = ros::Time();

    viz_pub.publish(pub_cloud);
}
#endif
