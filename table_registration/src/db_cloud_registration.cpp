#include <mongodb_store/message_store.h>
#include <geometry_msgs/Pose.h>
#include <zlib.h>

#include <boost/foreach.hpp>
#include <sensor_msgs/PointCloud2.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "db_cloud_registration");
    ros::NodeHandle nh;
    mongodb_store::MessageStoreProxy messageStore(nh,"ws_observations");
    std::vector< boost::shared_ptr<sensor_msgs::PointCloud2> > result_pc2;
    //tf info was pickled, need unpickle before use
    std::vector< boost::shared_ptr<mongodb_store_msgs::SerialisedMessage> > result_tf;

    //search all point clouds
    messageStore.query<sensor_msgs::PointCloud2>(result_pc2);

    std::cout<<result_pc2.size()<<std::endl;

    //BOOST_FOREACH( boost::shared_ptr<sensor_msgs::PointCloud2> pc2, result_pc2){
                    //ROS_INFO_STREAM("gooooo"<<*pc2);
                //}

    //get all the tf transformation & uncompress msg
    messageStore.query<mongodb_store_msgs::SerialisedMessage>(result_tf);
    //ROS_INFO_STREAM("goooooooo"<<*result_tf[0]);
    z_stream stream_tf;
    stream_tf.zalloc = Z_NULL;
    stream_tf.zfree = Z_NULL;
    stream_tf.opaque = Z_NULL;


    std::string compressed_str;
    for(int i=0;i<result_tf[0]->msg.size();i++){

        compressed_str+=result_tf[0]->msg[i];
        std::cout<<result_tf[0]->msg[i];
    }
    std::cout<<std::endl;

    uLongf outputsize=1000;
    std::vector<Bytef> output(1000);
    const Bytef *input = reinterpret_cast<const Bytef*>(compressed_str.c_str());
    int result;
    while((result=uncompress(&output.front(), &outputsize, input, compressed_str.length())) == Z_BUF_ERROR)
    {
        outputsize*=2;
        output.resize(outputsize);
    }

    std::cout<<compressed_str<<std::endl;


    for(int i=0;i<output.size();i++)
    {
        std::cout<<output[i];
    }


    // sort by time, match point cloud and tf transformation
    //result_pc2[0]->
    return 0;
}
