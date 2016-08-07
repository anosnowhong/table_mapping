#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

void map_analyzer(nav_msgs::OccupancyGrid &map)
{
    //create and init map data matrix
    std::vector<std::vector<int> > matrix_map;
    matrix_map.resize(map.info.width);
    for(int i=0;i<map.info.width;i++)
        matrix_map[i].resize(map.info.height);

    //convert map data to matrix
    int ros_map_cell=0;
    for(int i=0;i<map.info.width;i++) {
        for (int j = 0; j < map.info.height; j++) {
            matrix_map[i][j] = map.data[ros_map_cell];
            ros_map_cell++;
        }
    }

    cv::Mat cv_map(map.info.width, map.info.height, CV_8SC1);

    for(int i=0;i<map.info.width;i++)
    {
        for(int j=0;j<map.info.height;j++) {
            if(matrix_map[i][j] == 0)
            {
                cv_map.at<char>(i,j) = 0;
            }
            else
                cv_map.at<char>(i,j) = 1;
        }
    }
    std::cout<<cv_map<<std::endl;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "getmap");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<nav_msgs::GetMap>("static_map");

    nav_msgs::GetMap srv;
    nav_msgs::OccupancyGrid map_data;

    if (client.call(srv))
    {
        map_data = srv.response.map;
        map_analyzer(map_data);
    }
    else
    {
        ROS_ERROR("Failed to call service GetMap");
        return 1;
    }
    return 0;
}
