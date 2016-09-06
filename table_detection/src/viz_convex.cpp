
//viz convex from strands_perception_msg

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <mongodb_store/message_store.h>
#include <table_detection/Table.h>

typedef pcl::PointXYZ  Point;
typedef pcl::PointCloud<Point> pcl_cloud;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "viz_convex");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");


    std::string collection;
    std::vector<int> indices;
    pn.param<std::string>("collection",collection, "whole_table_shapes");
    pn.getParam("specify_index",indices);

    ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("table_convex", 1);

    mongodb_store::MessageStoreProxy table_store(n, collection);
    std::vector< boost::shared_ptr<table_detection::Table> > result_tables;
    table_store.query<table_detection::Table>(result_tables);

    //publish multiple point cloud msg
    std::vector<sensor_msgs::PointCloud2> cloud_msg;
    std::vector<pcl_cloud > cloud_arr;


    cloud_arr.resize(indices.size());
    cloud_msg.resize(indices.size());
    for(int j=0;j<indices.size();j++){

        int p_size = result_tables[indices[j]]->tabletop.points.size();

        for(int i=0;i<p_size;i++){
            Point pp;
            pp.x = result_tables[indices[j]]->tabletop.points.at(i).x;
            pp.y = result_tables[indices[j]]->tabletop.points.at(i).y;
            pp.z = result_tables[indices[j]]->tabletop.points.at(i).z;
            cloud_arr[j].push_back(pp);
        }
        pcl::toROSMsg(cloud_arr[j], cloud_msg[j]);
        //fill msg
        cloud_msg[j].header.stamp = ros::Time();
        cloud_msg[j].header.frame_id = "/map";
    }

    while(ros::ok())
    {
        ROS_INFO("Publishing %lu messages...", cloud_arr.size());
        for(int i=0;i<cloud_arr.size();i++){
            pub.publish(cloud_msg[i]);
        }
        sleep(2);
    }
    ros::spin();
}
