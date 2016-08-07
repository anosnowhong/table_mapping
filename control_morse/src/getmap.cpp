#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>

void map_analyzer(nav_msgs::OccupancyGrid &map)
{
    int rows = map.info.width;
    int cols = map.info.height;

    //create and init map data matrix
    std::vector<std::vector<int> > matrix_map;
    matrix_map.resize(cols);
    for(int i=0;i<cols;i++)
        matrix_map[i].resize(rows);

    //convert map data to matrix
    int ros_map_cell=0;
    for(int i=0;i<cols;i++) {
        for (int j = 0; j < rows; j++) {
            matrix_map[i][j] = map.data[ros_map_cell];
            ros_map_cell++;
        }
    }

    cv::Mat cv_map(cols, rows, CV_8UC1);
    //cv::Mat origin_map(map.info.width, map.info.height, CV_64F);

    for(int i=0;i<cols;i++)
    {
        for(int j=0;j<rows;j++) {
            if(matrix_map[i][j] == 0)
            {
                cv_map.at<char>(i,j) = 255;
                //origin_map.at<double>(i,j) = matrix_map[i][j];
            }
            else
            {
                cv_map.at<char>(i,j) = 0;
                //origin_map.at<double>(i,j) = matrix_map[i][j];
            }
        }
    }
    std::cout<<cv_map<<std::endl;

    /*
    std::ofstream matrix, cvmat;
    matrix.open("matrix.txt");
    matrix<<origin_map;
    matrix.close();
    cvmat.open("cvmat.txt");
    cvmat<<cv_map;
    cvmat.close();
     */

    cv::imwrite("cvimg.pgm",cv_map);
    cv::namedWindow( "ros occupancy map", cv::WINDOW_NORMAL);
    cv::imshow("ros occupancy map", cv_map);
    cv::waitKey(0);

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
