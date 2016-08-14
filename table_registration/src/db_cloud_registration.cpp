#include <mongodb_store/message_store.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
//#include <algorithm>
#include "table_registration/registration_operator.hpp"
#include <sensor_msgs/PointCloud2.h>


typedef boost::shared_ptr<geometry_msgs::TransformStamped>  TransformStampedPtr;
typedef boost::shared_ptr<sensor_msgs::PointCloud2> PointCloud2Ptr;

bool compare_tf(const TransformStampedPtr &x1, const TransformStampedPtr &x2)
{
    return (x1->header.seq<x2->header.seq);
}




int main(int argc, char** argv)
{
    ros::init(argc, argv, "db_cloud_registration");
    ros::NodeHandle nh;
    mongodb_store::MessageStoreProxy messageStore(nh,"ws_observations");
    std::vector< boost::shared_ptr<sensor_msgs::PointCloud2> > result_pc2;
    //tf info was pickled, need unpickle before use
    std::vector< boost::shared_ptr<geometry_msgs::TransformStamped> > result_tf;

    //search all point clouds
    messageStore.query<sensor_msgs::PointCloud2>(result_pc2);
    //get all the tf transformation
    messageStore.query<geometry_msgs::TransformStamped>(result_tf);

    std::cout<<"pc2: "<<result_pc2.size()<<std::endl;
    std::cout<<"tf: "<<result_tf.size()<<std::endl;

    //BOOST_FOREACH( boost::shared_ptr<sensor_msgs::PointCloud2> pc2, result_pc2){
                    //ROS_INFO_STREAM("gooooo"<<*pc2);
                //}

    //ROS_INFO_STREAM(*result_tf[0]);
    std::cout<<std::endl;

    //std::cout<<result_tf[0]->header.stamp<<std::endl;

    // sort by time, match point cloud and tf transformation
    //result_pc2[0]->
    //auto tmp_lambda = [](auto a, auto&&b){return a<b?a:b; };

    /*
    std::sort(result_tf.begin(), result_tf.end(),
              [](const TransformStampedPtr& x1, const TransformStampedPtr& x2){
        return (x1->header.seq < x2->header.seq);
    });
     */

    //std::sort(result_tf.begin(), result_tf.end(), compare_tf);

    result_tf[0]->transform.translation;
    result_tf[0]->transform.rotation;

    //trans_m = trans_to_matrix(result_tf[0]->transform.translation);
    //rot_m = rot_to_matrix(result_tf[0]->transform.rotation);


    //load point cloud data to pcl::PointCloud2



    //do the transformation things

    //save and visualization
    return 0;
}
