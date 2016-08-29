#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv,"heartbeat");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<std_msgs::String>("heartbeat",1000);
    ros::Rate loop_rate(100);

    std::clock_t start_t = std::clock();
    std::clock_t end_t = std::clock();
    while (ros::ok()){
        std::clock_t end_t = std::clock();

        std_msgs::String msg;
        std::stringstream ss;
        //log runing time
        ss<<"alive "<<end_t-start_t;
        msg.data = ss.str();
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
