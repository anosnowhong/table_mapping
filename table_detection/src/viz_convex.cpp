
//viz convex from strands_perception_msg

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <mongodb_store/message_store.h>
#include <table_detection/Table.h>
#include <table_detection/viz_points.h>

typedef pcl::PointXYZ  Point;
typedef pcl::PointCloud<Point> pcl_cloud;
ros::NodeHandlePtr n;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "centreview");
    n.reset(new ros::NodeHandle);
    ros::NodeHandle pn("~");


    std::string collection;
    std::vector<int> indices;
    pn.param<std::string>("collection",collection, "table_centre_group");
    pn.getParam("specify_index",indices);

    if(collection=="table_centre_group"){

        mongodb_store::MessageStoreProxy table_centre_group(*n, collection);
        std::vector<boost::shared_ptr<geometry_msgs::Polygon> > result_tables;
        table_centre_group.query<geometry_msgs::Polygon>(result_tables);

        VIZ_Points vv(n,"/table_centre_group");
        std::vector<float> color;
        color.push_back(0.0);
        color.push_back(1.0);
        color.push_back(0.0);

        VIZ_Points vv2(n,"/table_centre_group2");
        std::vector<float> color2;
        color2.push_back(0.0);
        color2.push_back(0.0);
        color2.push_back(1.0);

        VIZ_Points vv3(n,"/table_centre_group3");
        std::vector<float> color3;
        color3.push_back(1.0);
        color3.push_back(0.0);
        color3.push_back(0.0);

        while (ros::ok()){
            vv.pub_polygonASpoints(result_tables,0,color);

            vv2.pub_polygonASpoints(result_tables,1,color2);

            vv3.pub_polygonASpoints(result_tables,2,color3);
            sleep(1);
        }
        return 0;
    }

}
