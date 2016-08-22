#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <visualization_msgs/MarkerArray.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/flann/flann_base.hpp>
#include <opencv2/flann/miniflann.hpp>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <signal.h>

//#include <table_detection/ROIcloud.h>
#include <control_morse/WholeScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <control_morse/Pause.h>
#include <nav_msgs/Odometry.h>

#define PRINTOUTS true
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
ros::Publisher vis_pub;
ros::ServiceClient scan_srv;
ros::ServiceClient client;
control_morse::WholeScan scan_type;
bool state_pause=false;
int search_range;
int waiting_time;
MoveBaseClient *ac;

void merge_freecell(std::vector<cv::Point2i> &free_space)
{
    //free_space
}

void map_kdtree(std::vector<cv::Point2f> &pd_arr, cv::Mat &cv_map,  std::vector<cv::Point2i> &free_space, int search_range)
{
    cv::flann::KDTreeIndexParams index_params;
    //float vector for kdtree
    cv::flann::Index kdtree(cv::Mat(pd_arr).reshape(1),index_params);

    int radius=search_range; //pixel
    double radius_square = pow(search_range*0.05, 2);//meter = (x1-x2)^2 + (y1-y2)^2
    //query point
    cv::Point2f p2d;
    p2d.x=p2d.y=0.0;
    std::vector<float> query;
    std::vector<int> indices;
    //squared distance
    std::vector<float> dists;

    //traverse available space
    for(int i=0;i<cv_map.cols;i++)
    {
        for(int j=0;j<cv_map.rows;j++)
        {
            p2d.x= j*0.05;
            p2d.y= i*0.05;
            query.push_back(p2d.x);
            query.push_back(p2d.y);
            kdtree.radiusSearch(query,indices,dists,radius_square,100,cv::flann::SearchParams(64));
            //can find an occupied cell
            if(indices[1] == 0)
            {
                //find a free area
                std::cout<<"rows: "<<j<<"cols: "<<i<<std::endl;
                free_space.push_back(cv::Point2i(i,j));
            }

            //Debug push all the cell
            //free_space.push_back(cv::Point2i(i,j));

            j+=search_range;//skip 20 pixel(search radius)
            indices.clear();
            dists.clear();
            query.clear();

        }
        i+=search_range;
    }

    if(PRINTOUTS)
    {
        std::cout<<"Free space num: "<<free_space.size()<<std::endl;
        //std::copy(indices.begin(),indices.end(),std::ostream_iterator<int>(std::cout, " "));
        //std::cout<<std::endl;
        //std::copy(dists.begin(),dists.end(),std::ostream_iterator<float>(std::cout, " "));
    }

}

//generate goal index
void map_index(double origin[2], float res, int width, int height, std::vector<cv::Point2i> &free_space, int search_range)
{
    double index_bob =  0.8; //bob radius * 2
    double w_meter = width*res;
    double h_meter = height*res;
    double max_num_w = w_meter/index_bob;
    double max_num_h = h_meter/index_bob;

    //viz_markers(origin, free_space, height);

    visualization_msgs::MarkerArray mark_arr;
    mark_arr.markers.resize(free_space.size());
    for(int i=0;i<free_space.size();i++)
    {
        mark_arr.markers[i].header.frame_id="/map";
        mark_arr.markers[i].header.stamp = ros::Time();
        mark_arr.markers[i].ns = "waypoint";
        mark_arr.markers[i].id = i;
        mark_arr.markers[i].type=visualization_msgs::Marker::CYLINDER;
        mark_arr.markers[i].action= visualization_msgs::Marker::ADD;
        //in meter
        mark_arr.markers[i].pose.position.x = origin[0]+free_space[i].x*0.05;
        //upside down
        mark_arr.markers[i].pose.position.y = origin[1]+(height-free_space[i].y)*0.05;
        mark_arr.markers[i].pose.position.z = 1;
        mark_arr.markers[i].pose.orientation.x = 0;
        mark_arr.markers[i].pose.orientation.y = 0;
        mark_arr.markers[i].pose.orientation.z = 0;
        mark_arr.markers[i].pose.orientation.w = 1;

        //x=y=1 => 1meter,
        double ratio = (search_range*0.05*2)/1.0;//(radius * 2)/default_marker_length
        mark_arr.markers[i].scale.x = 0.5*ratio;
        mark_arr.markers[i].scale.y = 0.5*ratio;
        mark_arr.markers[i].scale.z = 0.1;
        mark_arr.markers[i].color.a = 1.0;
        mark_arr.markers[i].color.r = 1.0;
        mark_arr.markers[i].color.g = 0.0;
        mark_arr.markers[i].color.b = 0.0;
    }
    vis_pub.publish(mark_arr);

    //publish index positon one by one
    for(int i=0;i<free_space.size();i++)
    {
        move_base_msgs::MoveBaseGoal goal_index;
        goal_index.target_pose.header.frame_id="/map";
        goal_index.target_pose.header.stamp=ros::Time();

        std::cout<<free_space[i].x<<" "<<free_space[i].y<<std::endl;
        goal_index.target_pose.pose.position.x = origin[0] + free_space[i].x * res;
        goal_index.target_pose.pose.position.y = origin[1] + (height-free_space[i].y) * res;
        goal_index.target_pose.pose.orientation.w=1;

        while(!ac->waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up...");
        }

        ac->sendGoal(goal_index);
        ROS_INFO("Goal has been send!( %G, %G)", goal_index.target_pose.pose.position.x,
                 goal_index.target_pose.pose.position.y);
        //set time to wait
        ac->waitForResult(ros::Duration(waiting_time));

        if(ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("goal succeeded, scanning...");
            //grab_srv.call(grab_cloud);
            scan_type.request.scan_type="whole";
            scan_srv.call(scan_type);
            scan_srv.waitForExistence();
        }
        else
        {
            ROS_INFO("goal failed");
            ac->cancelGoal();
        }
    }

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

void sig_shutdown(int sig)
{
    ros::shutdown();
}

//convert ros map data to cv mat
void map_analyzer(nav_msgs::OccupancyGrid &map, int search_range)
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

    //cv::imwrite("cvimg.pgm",cv_map);
    //cv::imshow("ros occupancy map", cv_map);
    //cv::waitKey(0);

    std::vector<cv::Point2f> map_points;
    cvmat2points(cv_map, map_points);
    std::vector<cv::Point2i> free_index;
    map_kdtree(map_points, cv_map, free_index, search_range);

    //map info that stored in *.yaml file
    float res = map.info.resolution;
    double origin_map[2];
    origin_map[0]= map.info.origin.position.x;
    origin_map[1]= map.info.origin.position.y;
    map_index(origin_map,res,cols,rows,free_index,search_range);

    //sig_shutdown(SIGINT);
}

bool pause_action(control_morse::Pause::Request &req, control_morse::Pause::Response & res)
{
    state_pause = req.Paused;
    ROS_INFO("Pause Flag has been set");
    return true;
}

//just a loop thread for pause_action
void pause_thread(sensor_msgs::PointCloud2 d1)
{
    //if pause service is called, then stop action
    if(state_pause)
    {
        ROS_INFO("puase request received, pausing robot...");
        //MoveBaseClient ac_pause("move_base", true);
        //move_base_msgs::MoveBaseGoal empty_goal;
        //ac_pause.sendGoal(empty_goal);
        ac->cancelGoal();
        sleep(1);
        state_pause=false;
    }
    else
    {
        sleep(1);
    }
}

void main_thread(sensor_msgs::PointCloud2 d2)
{

    nav_msgs::GetMap srv;
    nav_msgs::OccupancyGrid map_data;

    while (ros::ok())
    {
        if (client.call(srv))
        {
            map_data = srv.response.map;
            map_analyzer(map_data, search_range);
        }
        else
        {
            ROS_ERROR("Failed to call service GetMap");
        }

        sleep(1);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_analyzer");
    ros::NodeHandle n;
    signal(SIGINT, sig_shutdown);
    ros::NodeHandle pn("~");

    ac = new MoveBaseClient("move_base", true);

    ros::MultiThreadedSpinner spinner(3);
    //get params
    pn.param<int>("search_range", search_range, 20);
    pn.param<int>("goal_waiting_time", waiting_time, 60);
    //pn.getParam("/getmap/search_range", search_range);

    vis_pub = n.advertise<visualization_msgs::MarkerArray>("/waypoints", 1);
    ros::ServiceServer server = n.advertiseService("pause_morse", pause_action);
    scan_srv = n.serviceClient<control_morse::WholeScan>("do_scan");
    client  = n.serviceClient<nav_msgs::GetMap>("static_map");

    ros::Subscriber sub = n.subscribe("/head_xtion/depth/points",1,pause_thread);
    ros::Subscriber sub2 = n.subscribe("/head_xtion/depth/points",1,main_thread);

    std::cout<<"using search range:"<<search_range<<std::endl;

    spinner.spin();
}
