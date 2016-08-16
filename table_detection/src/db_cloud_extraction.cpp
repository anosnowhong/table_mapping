
#include <mongodb_store/message_store.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <primitive_extraction/ExtractPrimitives.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>

/* #A backend node that does the extraction work with MongoDB.
 * Load point cloud data from global_cloud collection.
 * call the extraction service
 * use table parameter to filer out table plane
 * store to table_plane collection
 * */
typedef boost::shared_ptr<geometry_msgs::TransformStamped>  TransformStampedPtr;
typedef boost::shared_ptr<sensor_msgs::PointCloud2> PointCloud2Ptr;

#define Debug true

int main(int argc, char** argv)
{
/*
    ros::init(argc, argv, "db_cloud_registration");
    ros::NodeHandle nh;

    ros::ServiceClient client=nh.serviceClient<primitive_extraction::ExtractPrimitives>("extract_primitives");
    //query point cloud and tf
    mongodb_store::MessageStoreProxy messageStore(nh,"global_clouds");
    //store point cloud that been transformed
    mongodb_store::MessageStoreProxy cloud_store(nh,"table_planes");
    std::vector< boost::shared_ptr<sensor_msgs::PointCloud2> > result_pc2;

    //search all point clouds
    messageStore.query<sensor_msgs::PointCloud2>(result_pc2);

    std::cout<<"pc2: "<<result_pc2.size()<<std::endl;
    //std::cout<<"tf: "<<result_tf.size()<<std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr db_global_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr db_store(new pcl::PointCloud<pcl::PointXYZ>());
    sensor_msgs::PointCloud2 db_store_msg;

    for(int i=0;i<result_pc2.size();i++)
    {
        //reset
        db_global_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        db_store.reset(new pcl::PointCloud<pcl::PointXYZ>);

        //load point cloud data to pcl::PointCloud2
        pcl::fromROSMsg(*result_pc2[i],*db_global_cloud);
        //call the extraction service


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
    */
    return 0;
}