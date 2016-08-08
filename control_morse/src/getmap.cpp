#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "control_morse/Quadtree.hpp"

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//generate goal index
void map_index(double origin[2], float res, int width, int height)
{
    double index_bob =  0.8; //bob radius * 2
    double w_meter = width*res;
    double h_meter = height*res;
    double max_num_w = w_meter/index_bob;
    double max_num_h = h_meter/index_bob;

    MoveBaseClient ac("move_base", true);
    move_base_msgs::MoveBaseGoal goal_index;
    goal_index.target_pose.header.frame_id="/map";
    goal_index.target_pose.header.stamp=ros::Time();

    goal_index.target_pose.pose.position.x = 3.0;
    goal_index.target_pose.pose.position.y = origin[2] + index_bob;
    goal_index.target_pose.pose.orientation.w=1;

    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up...");
    }


    ac.sendGoal(goal_index);
    ROS_INFO("Goal has been send!( %G, %G)", goal_index.target_pose.pose.position.x,
             goal_index.target_pose.pose.position.y);
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("goal succeeded");
    else
        ROS_INFO("goal failed");
}


//convert ros map data to cv mat
void map_analyzer(nav_msgs::OccupancyGrid &map)
{
    int rows = map.info.height;
    int cols = map.info.width;

    //create and init map data matrix
    std::vector<std::vector<int> > matrix_map;
    matrix_map.resize(rows);
    for(int i=0;i<rows;i++)
        matrix_map[i].resize(cols);

    //convert map data to matrix
    int ros_map_cell=0;
    for(int i=rows-1;i>=0;i--) {
        for (int j = 0; j<cols; j++) {
            matrix_map[i][j] = map.data[ros_map_cell];
            ros_map_cell++;
        }
    }

    cv::Mat cv_map(rows, cols, CV_8UC1);

    for(int i=0;i<rows;i++)
    {
        for(int j=0;j<cols;j++) {
            if(matrix_map[i][j] == 0)
            {
                //255 is white
                cv_map.at<char>(i,j) = 255;
                //std::cout<<'1';
            }
            else
            {
                //0 is black, gray for between 0-255
                cv_map.at<char>(i,j) = 0;
                //std::cout<<'0';
            }

        }
        //std::cout<<std::endl;
    }

    //cv::imwrite("cvimg.pgm",cv_map);
    //cv::imshow("ros occupancy map", cv_map);
    //cv::waitKey(0);
    Quadtree qtree(0,0,100,100,0,2);


    float res = map.info.resolution;
    double origin_map[2];
    origin_map[1]= map.info.origin.position.x;
    origin_map[2]= map.info.origin.position.y;
    map_index(origin_map,res,cols, rows);

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
