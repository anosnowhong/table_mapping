#include <ros/ros.h>
#include <control_morse/WholeScan.h>
#include <actionlib/client/simple_action_client.h>
#include <scitos_ptu/PtuGotoAction.h>
#include <table_detection/ROIcloud.h>

typedef actionlib::SimpleActionClient<scitos_ptu::PtuGotoAction> PTU_Client;
PTU_Client *ptu;
ros::ServiceClient grab_srv;
table_detection::ROIcloud grab_cloud;

bool whole_scan(control_morse::WholeScan::Request &req,
                control_morse::WholeScan::Response &res)
{
    while(!ptu->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the PTU action server to come up...");
    }

    //TODO:this is special control for PTU in morse only
    if(req.scan_type=="whole")
    {
        float pan_interval = 60.0;
        for(int i=0;i<360.0/pan_interval  ;i++)
        {
            //define action msg
            scitos_ptu::PtuGotoGoal ptu_goal;

            ptu_goal.pan_vel=100.0;
            ptu_goal.tilt_vel=100.0;

            ptu_goal.pan= -i*pan_interval + 170;// can't reach 180, keep turning
            ptu_goal.tilt=30.0;

            //move ptu
            ptu->sendGoal(ptu_goal);

            ptu->waitForResult();
            if(ptu->getState()==actionlib::SimpleClientGoalState::SUCCEEDED){
                ROS_INFO("PTU goal succeeded.");
                grab_srv.call(grab_cloud);//return the grabed cloud
                sleep(1);
            }
            else{
                ROS_INFO("PTU goal failed.");
                sleep(1);
            }
        }

    }
    else if(req.scan_type=="single")
    {
        grab_srv.call(grab_cloud);
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ptu_control");
    ros::NodeHandle n;
    ros::ServiceServer server = n.advertiseService("do_scan",whole_scan);

    //service client
    grab_srv = n.serviceClient<table_detection::ROIcloud>("ROIcloud");
    //action client
    ptu = new PTU_Client("/SetPTUState", true);

    ros::spin();
}
