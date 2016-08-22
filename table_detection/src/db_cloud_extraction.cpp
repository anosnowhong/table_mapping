#include <mongodb_store/message_store.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <primitive_extraction/ExtractPrimitives.h>
#include <pcl_ros/point_cloud.h>
#include <table_detection/db_extract.h>
#include <table_detection/db_extract_whole_table.h>
#include <primitives_to_tables/PrimitivesToTables.h>
#include <strands_perception_msgs/Table.h>
#include <std_msgs/Int32.h>

/* #A backend node that does the extraction work with MongoDB.
 * Load point cloud data from global_cloud collection.
 * call the extraction service
 * use table parameter to filer out table plane
 * store to table_plane collection
 * */

ros::ServiceClient primitive_client;
ros::NodeHandlePtr nh;

double min_height;
double max_height;
double max_angle;
double min_side_ratio;
double min_area;

int checked_num=0;

#define Debug false

double compute_plane_area(std::vector<geometry_msgs::Point>& hull)
{
    double area = 0.0;
    // assume that it's almost parallell to the ground
    Eigen::Vector2d p0(hull[0].x, hull[0].y);
    for (size_t i = 1; i < hull.size()-1; ++i) {
        Eigen::Vector2d p1(hull[i].x, hull[i].y);
        Eigen::Vector2d p2(hull[i+1].x, hull[i+1].y);
        // calculate area by Heron's formula
        // this can be done by det as in table_tracking
        double a = (p1-p0).norm();
        double b = (p2-p0).norm();
        double c = (p2-p1).norm();
        double s = 0.5*(a + b + c);
        area += sqrt(s*(s-a)*(s-b)*(s-c));
    }
    return area;
}

void table_filter(primitive_extraction::PrimitiveArray &input,primitive_extraction::PrimitiveArray &output)
{
    output.camera_frame = input.camera_frame;
    size_t n = input.primitives.size();

    for (size_t i = 0; i < n; ++i) {
        primitive_extraction::Primitive p = input.primitives[i];

        // check normal
        double alpha = acos(fabs(p.params[4]));
        if (alpha > max_angle) {
            //ROS_INFO("Stopped because of angle: %f", alpha);
            continue;
        }

        double camera_height = 0.0;
        // check height
        double height = camera_height + p.pose.position.z;
        if (height < min_height || height > max_height) {
            //ROS_INFO("Stopped because of height: %f", height);
            continue;
        }

        // check size
        double area = compute_plane_area(p.points);
        if (area < min_area) {
            //ROS_INFO("Stopped because of area: %f", area);
            continue;
        }

        // check shape
        double minside, maxside;
        if (p.params[0] > p.params[1]) {
            minside = p.params[1];
            maxside = p.params[0];
        }
        else {
            minside = p.params[0];
            maxside = p.params[1];
        }

        double ratio = minside/maxside;
        if (ratio < min_side_ratio) {
            //ROS_INFO("Stopped because of ratio: %f", ratio);
            continue;
        }

        output.primitives.push_back(p);
        //indices.push_back(i);
    }
}

bool extract(table_detection::db_extract::Request &req, table_detection::db_extract::Response &res)
{
    ROS_INFO("Starting extract table");
    //query point cloud and tf
    mongodb_store::MessageStoreProxy global_clouds(*nh,"global_clouds");
    //store point cloud that been transformed
    mongodb_store::MessageStoreProxy table_store(*nh,"table_planes");
    std::vector< boost::shared_ptr<sensor_msgs::PointCloud2> > result_pc2;

    //search all point clouds
    global_clouds.query<sensor_msgs::PointCloud2>(result_pc2);

    //load check_num form mongodb, this value record how many global clouds has been checked.
    std::vector< boost::shared_ptr<std_msgs::Int32> > checked_amount;
    table_store.queryNamed<std_msgs::Int32>("checked_num", checked_amount);
    if(checked_amount.size() == 0){
        std_msgs::Int32 tmp;
        tmp.data = checked_num;
        table_store.insertNamed("checked_num",tmp);
    }
    else{
        //load record form mongodb
        checked_num = checked_amount[0]->data;
    }

    ROS_INFO("Cloud found: %lu, already checked: %d", result_pc2.size(), checked_num);

    for(;checked_num<result_pc2.size();checked_num++)
    {
        //call the extraction service,get primitives planes
        primitive_client.waitForExistence();
        primitive_extraction::ExtractPrimitives req;
        req.request.pointcloud=*result_pc2[checked_num];
        primitive_client.call(req);

        //filter out possible table planes
        primitive_extraction::PrimitiveArray to_table;
        table_filter(req.response.primitives,to_table);

        //fill the table msg
        std::vector<strands_perception_msgs::Table> tables;
        tables.resize(to_table.primitives.size());
        for (size_t i = 0; i < to_table.primitives.size(); ++i) {
            strands_perception_msgs::Table table;
            primitive_extraction::Primitive primitive = to_table.primitives[i];
            table.header.frame_id = to_table.camera_frame;
            table.header.stamp = result_pc2[checked_num]->header.stamp;
            table.pose.pose = primitive.pose;
            table.tabletop.points.resize(primitive.points.size());
            for (size_t j = 0; j < primitive.points.size(); ++j) {
                table.tabletop.points[j].x = primitive.points[j].x;
                table.tabletop.points[j].y = primitive.points[j].y;
                table.tabletop.points[j].z = primitive.points[j].z;
            }
            tables[i] = table;
            //insert it to mongodb
            table_store.insert(table);
        }

        //store the index of point cloud as well
        if(to_table.primitives.size()>0)
            res.cloud_has_plane.push_back(checked_num);
    }
    //update check_num in mongodb
    std_msgs::Int32 tmp;
    tmp.data = checked_num;
    table_store.updateNamed("checked_num",tmp);

    if(Debug)
    {
        for(int i=0;i<res.cloud_has_plane.size();i++)
            std::cout<<res.cloud_has_plane[i]<<std::endl;
    }
    return true;
}

bool extract_whole_table(table_detection::db_extract_whole_table::Request &req, table_detection::db_extract_whole_table::Response &res)
{
    ROS_INFO("Starting extract whole table");
    mongodb_store::MessageStoreProxy whole_tables(*nh,"whole_tables");

    //call the extraction service,get primitives planes
    primitive_extraction::ExtractPrimitives primitives_req;
    primitives_req.request.pointcloud=req.cloud;
    primitive_client.call(primitives_req);

    //filter out possible table planes
    primitive_extraction::PrimitiveArray to_table;
    table_filter(primitives_req.response.primitives,to_table);

     std::vector<strands_perception_msgs::Table> tables;
        tables.resize(to_table.primitives.size());
        for (size_t i = 0; i < to_table.primitives.size(); ++i) {
            strands_perception_msgs::Table table;
            primitive_extraction::Primitive primitive = to_table.primitives[i];
            table.header.frame_id = to_table.camera_frame;
            //table.header.stamp = result_pc2[checked_num]->header.stamp;
            table.pose.pose = primitive.pose;
            table.tabletop.points.resize(primitive.points.size());
            for (size_t j = 0; j < primitive.points.size(); ++j) {
                table.tabletop.points[j].x = primitive.points[j].x;
                table.tabletop.points[j].y = primitive.points[j].y;
                table.tabletop.points[j].z = primitive.points[j].z;
            }
            tables[i] = table;
            //insert it to mongodb
            whole_tables.insert(table);
        }
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "db_cloud_extraction");
    nh.reset(new ros::NodeHandle);
    ros::NodeHandle pn("~");

    pn.param<double>("min_height", min_height, 0.55);
    pn.param<double>("max_height", max_height, 1.1);
    pn.param<double>("max_angle", max_angle, 0.314);
    pn.param<double>("min_side_ration", min_side_ratio, 0.25);
    pn.param<double>("min_area", min_area, 0.3);

    primitive_client=nh->serviceClient<primitive_extraction::ExtractPrimitives>("extract_primitives");
    ros::ServiceServer ext_srv = nh->advertiseService("db_extract", extract);
    ros::ServiceServer ext_srv2 = nh->advertiseService("db_extract_whole_table", extract_whole_table);

    ros::spin();
}