#include <mongodb_store/message_store.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include "table_registration/registration_operator.hpp"
#include <table_registration/ToGlobal.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>


/* #A backend node that does the registration work with MongoDB.
 * Load point cloud and tf data from ws_observation collection.
 * Convert to global coordinate system (/map)
 * Stored to a new collection.
 * */
typedef boost::shared_ptr<geometry_msgs::TransformStamped>  TransformStampedPtr;
typedef boost::shared_ptr<sensor_msgs::PointCloud2> PointCloud2Ptr;

#define Debug false

ros::NodeHandlePtr nh;
bool regist = false;

//service call to convert cloud in ws_observation to global_cloud collection
bool to_global(table_registration::ToGlobal::Request &req, table_registration::ToGlobal::Response &res)
{
    //query point cloud and tf
    mongodb_store::MessageStoreProxy messageStore(*nh,"ws_observations");
    //store point cloud that been transformed
    mongodb_store::MessageStoreProxy cloud_store(*nh,"global_clouds");

    std::vector< boost::shared_ptr<sensor_msgs::PointCloud2> > result_pc2;
    std::vector< boost::shared_ptr<sensor_msgs::PointCloud2> > result_global_pc2;
    std::vector< boost::shared_ptr<geometry_msgs::TransformStamped> > result_tf;

    //search all point clouds
    messageStore.query<sensor_msgs::PointCloud2>(result_pc2);
    //get all the tf transformation
    messageStore.query<geometry_msgs::TransformStamped>(result_tf);
    //get all transformed point clouds
    cloud_store.query<sensor_msgs::PointCloud2>(result_global_pc2);

    //how many clouds need to be transformed
    int queue = result_pc2.size()-result_global_pc2.size();
    int stored_num = result_pc2.size();
    if (queue == 0){
        ROS_INFO("Already transformed!");
        return false;
    }

    std::cout<<queue<<std::endl;
    std::cout<<"pc2: "<<result_pc2.size();
    std::cout<<" tf: "<<result_tf.size()<<std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr db_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr db_store(new pcl::PointCloud<pcl::PointXYZ>());
    sensor_msgs::PointCloud2 db_store_msg;
    geometry_msgs::Vector3 trans;
    geometry_msgs::Quaternion qua;
    for(int i=0 ;i<queue;i++)
    {
        //reset
        db_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        db_store.reset(new pcl::PointCloud<pcl::PointXYZ>);

        trans = result_tf[i+stored_num-queue]->transform.translation;
        qua = result_tf[i+stored_num-queue]->transform.rotation;
        //load point cloud data to pcl::PointCloud2
        pcl::fromROSMsg(*result_pc2[i+stored_num-queue],*db_cloud);
        //do the transformation things
        registration_operator<pcl::PointXYZ> reg_op(db_cloud,trans,qua);
        reg_op.to_global(*db_store);

        if(Debug){
            std::string name="/home/parallels/debug/.pcd";
            std::stringstream ss;
            ss<< i;
            std::string str = ss.str();
            name.insert(name.length()-4, str);
            std::cout<<name<<std::endl;
            pcl::io::savePCDFileASCII(name, *db_store);
        }

        //convert to msg and store it
        pcl::toROSMsg(*db_store,db_store_msg);

        //keep same header but different frame_id
        db_store_msg.header=result_pc2[i+stored_num-queue]->header;
        db_store_msg.header.frame_id = "/map";
        cloud_store.insert(db_store_msg);

    }
    ROS_INFO("Done. Point Clouds has been transformed to global coordinate '/map'");

    return true;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "db_cloud_registration");
    nh.reset(new ros::NodeHandle);
    ros::ServiceServer service = nh->advertiseService("to_global",to_global);

    ros::spin();
}