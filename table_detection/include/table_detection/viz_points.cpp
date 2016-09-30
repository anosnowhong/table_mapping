#include "viz_points.h"

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
