#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <mongodb_store/message_store.h>
#include <table_registration/ICP_Cloud.h>
#include <table_detection/table_neighbour_arr.h>
#include <geometry_msgs/Polygon.h>
#include <ros/ros.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "showpolygen");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");
    std::string collection;
    std::vector<int> indices;
    pn.param<std::string>("collection", collection, "global_clouds");
    pn.getParam("specify_index",indices);

    mongodb_store::MessageStoreProxy messageStore(n,"/table_centre_group");
    std::vector< boost::shared_ptr<geometry_msgs::Polygon> > result;
    messageStore.query<geometry_msgs::Polygon>(result);


    ROS_INFO("Loading %lu Point Clouds!",result.size());


    for(int j=0;j<result[0]->points.size();j++){
        std::cout<<"====";
        std::cout<<result[0]->points.at(j).x;
    }

    while (ros::ok()){
        sleep(1);
    }
}
