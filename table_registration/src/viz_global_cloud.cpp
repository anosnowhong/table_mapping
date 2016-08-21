
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <mongodb_store/message_store.h>

#define Debug false

int main(int argc, char** argv)
{
    ros::init(argc, argv, "viz_global");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");
    ros::Rate loop_rate(1);

    std::string collection;
    pn.param<std::string>("collection", collection, "global_clouds");

    ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("/global_cloud",10);
    mongodb_store::MessageStoreProxy messageStore(n,collection);
    std::vector< boost::shared_ptr<sensor_msgs::PointCloud2> > result_pc2;
    messageStore.query<sensor_msgs::PointCloud2>(result_pc2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sum(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_store(new pcl::PointCloud<pcl::PointXYZ>());
    ROS_INFO("Loading %lu Point Clouds!",result_pc2.size());
    for(int i=0;i<result_pc2.size();i++)
    {
        pcl::fromROSMsg(*result_pc2[i], *cloud_store);
        *cloud_sum += *cloud_store;

         if(Debug){
            std::string name="/home/parallels/debug/.pcd";
            std::stringstream ss;
            ss<< i;
            std::string str = ss.str();
            name.insert(name.length()-4, str);
            std::cout<<name<<std::endl;
            pcl::io::savePCDFileASCII(name, *cloud_store);
        }
    }
    sensor_msgs::PointCloud2 pub_cloud;
    pcl::toROSMsg(*cloud_sum,pub_cloud);

    //fill the missing info and make it latest time for Rviz to view
    pub_cloud.header = result_pc2[0]->header;
    pub_cloud.header.stamp = ros::Time();

    if(Debug){
        mongodb_store::MessageStoreProxy tt(n,"test");
        tt.insert(pub_cloud);
    }

    while(ros::ok())
    {
        pub.publish(pub_cloud);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
