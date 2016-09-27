#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <mongodb_store/message_store.h>
#include <table_registration/ICP_Cloud.h>
#include <table_detection/table_neighbour_arr.h>
#include <geometry_msgs/Polygon.h>
#include <table_detection/Table.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "showpolygen");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");
    std::string collection;
    std::vector<int> indices;
    pn.param<std::string>("collection", collection, "global_clouds");
    pn.getParam("specify_index",indices);
    ros::Publisher pub = n.advertise<geometry_msgs::Polygon>("/concave_polygon",10);

    mongodb_store::MessageStoreProxy messageStore(n,collection);
    //std::vector< boost::shared_ptr<sensor_msgs::PointCloud2> > result;
    std::vector< boost::shared_ptr<table_detection::Table> > result;
    messageStore.query<table_detection::Table>(result);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_store(new pcl::PointCloud<pcl::PointXYZ>());

    geometry_msgs::Polygon po;
    geometry_msgs::Point32 pp;

    ROS_INFO("Loading %lu Point Clouds!",indices.size());
    for(int i=0;i<indices.size();i++) {
        //pcl::fromROSMsg(*result[indices[i]], *cloud_store);
        ROS_INFO("Loading %lu Point Clouds!",indices.size());
        po = result[indices[i]]->tabletop;
        ROS_INFO("Loading %lu Point Clouds!",indices.size());
        //pp.x = cloud_store->points.at(i).x;
        //pp.y = cloud_store->points.at(i).y;

        //po.points.push_back(pp);
    }
    while (ros::ok()){
        ROS_INFO("Loading %lu Point Clouds!",indices.size());
        pub.publish(po);
        sleep(1);
    }
}
