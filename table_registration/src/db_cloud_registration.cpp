#include <mongodb_store/message_store.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
//#include <algorithm>
#include "table_registration/registration_operator.hpp"
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>

/* Load point cloud and tf data from ws_observation collection.
 * Convert to global coordinate system (/map)
 * Stored to a new collection.
 * */
typedef boost::shared_ptr<geometry_msgs::TransformStamped>  TransformStampedPtr;
typedef boost::shared_ptr<sensor_msgs::PointCloud2> PointCloud2Ptr;

#define Debug true

bool compare_tf(const TransformStampedPtr &x1, const TransformStampedPtr &x2)
{
    return (x1->header.seq<x2->header.seq);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "db_cloud_registration");
    ros::NodeHandle nh;

    //query point cloud and tf
    mongodb_store::MessageStoreProxy messageStore(nh,"ws_observations");
    //store point cloud that been transformed
    mongodb_store::MessageStoreProxy cloud_store(nh,"global_clouds");
    std::vector< boost::shared_ptr<sensor_msgs::PointCloud2> > result_pc2;
    std::vector< boost::shared_ptr<geometry_msgs::TransformStamped> > result_tf;

    //search all point clouds
    messageStore.query<sensor_msgs::PointCloud2>(result_pc2);
    //get all the tf transformation
    messageStore.query<geometry_msgs::TransformStamped>(result_tf);

    std::cout<<"pc2: "<<result_pc2.size()<<std::endl;
    std::cout<<"tf: "<<result_tf.size()<<std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr db_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr db_store(new pcl::PointCloud<pcl::PointXYZ>());
    sensor_msgs::PointCloud2 db_store_msg;
    geometry_msgs::Vector3 trans;
    geometry_msgs::Quaternion qua;

    for(int i=0;i<result_tf.size();i++)
    {
        //reset
        db_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        db_store.reset(new pcl::PointCloud<pcl::PointXYZ>);

        trans = result_tf[i]->transform.translation;
        qua = result_tf[i]->transform.rotation;
        //load point cloud data to pcl::PointCloud2
        pcl::fromROSMsg(*result_pc2[i],*db_cloud);
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
        db_store_msg.header=result_pc2[i]->header;
        db_store_msg.header.frame_id = "/map";
        cloud_store.insert(db_store_msg);

    }


    return 0;
}
