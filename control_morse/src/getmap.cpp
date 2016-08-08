#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/flann/flann_base.hpp>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <opencv2/flann/miniflann.hpp>

#define PRINTOUTS true
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void map_kdtree(std::vector<cv::Point2f> &pd_arr, cv::Mat &cv_map)
{
    cv::flann::KDTreeIndexParams index_params;
    //float vector for kdtree
    cv::flann::Index kdtree(cv::Mat(pd_arr).reshape(1),index_params);

    //int radius=10; //pixel
    double radius_square = pow(20*0.05, 2);//meter = (x1-x2)^2 + (y1-y2)^2
    //query point
    cv::Point2f p2d;
    p2d.x=p2d.y=0.0;
    std::vector<float> query;

    std::vector<int> indices;
    //squared distance
    std::vector<float> dists;
    std::vector<cv::Point2i> free_space;

    //traverse available space
    for(int i=0;i<cv_map.cols;i++)
    {
        //std::cout<<"================: "<<i<<std::endl;
        for(int j=0;j<cv_map.rows;j++)
        {
            p2d.x= j*0.05;
            p2d.y= i*0.05;
            //std::cout<<"x: "<<p2d.x<<"y: "<<p2d.y<<std::endl;
            query.push_back(p2d.x);
            query.push_back(p2d.y);
            kdtree.radiusSearch(query,indices,dists,radius_square,100,cv::flann::SearchParams(64));
            //can find an occupied cell
            if(indices[1] == 0)
            {
                //find a free area
                std::cout<<"rows: "<<j<<"cols: "<<i<<std::endl;
                free_space.push_back(cv::Point2i(j,i));
                //occupy this free area
            }

            j+=20;//skip 10 pixel(search radius)
            indices.clear();
            dists.clear();
            query.clear();

        }
        i+=20;
    }

    if(PRINTOUTS)
    {
        std::cout<<"Free space num: "<<free_space.size()<<std::endl;
        std::copy(indices.begin(),indices.end(),std::ostream_iterator<int>(std::cout, " "));
        std::cout<<std::endl;
        std::copy(dists.begin(),dists.end(),std::ostream_iterator<float>(std::cout, " "));
    }

}

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

//convert mat to 2D points(in meter)
void cvmat2points(cv::Mat &mm, std::vector<cv::Point2f> &pd_arr)
{
    cv::Point2f tmp;
    for(int i=0;i<mm.rows;i++)
    {
        for(int j=0;j<mm.cols;j++) {
            //1 means free cell
            if (mm.at<char>(i, j) == 0)
            {
                //usr index as coordinate
                tmp.x=i * 0.05;
                tmp.y=j * 0.05;
                pd_arr.push_back(tmp);
            }
        }
    }
    if(PRINTOUTS)
    {
        //ROS_INFO("%u",mm.at<char>(i,j));
        //std::cout<<"points are: "<<pd_arr<<std::endl;
        std::cout<<"Got "<<pd_arr.size()<<" occupied points in map."<<std::endl;
    }
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

    //convert matrix to 0 and 1 cv::Mat
    for(int i=0;i<rows;i++)
    {
        for(int j=0;j<cols;j++) {
            if(matrix_map[i][j] == 0)
            {
                //255 is white, 1 for convenient
                cv_map.at<char>(i,j) = 1;
                //std::cout<<'1';
            }
            else
            {
                //0 is black, gray for between 0-255
                cv_map.at<char>(i,j) = 0;
                //std::cout<<'0';
            }
        }
    }

    cv::imwrite("cvimg.pgm",cv_map);
    //cv::imshow("ros occupancy map", cv_map);
    //cv::waitKey(0);

    std::vector<cv::Point2f> map_points;
    cvmat2points(cv_map, map_points);
    map_kdtree(map_points, cv_map);
    return;

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
