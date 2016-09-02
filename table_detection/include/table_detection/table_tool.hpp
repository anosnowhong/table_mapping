#include "table_detection/table_tool.h"

template <class Point>
Table<Point>::Table(){

}

template <class Point>
Table<Point>::Table(ros::NodeHandlePtr nh_in) {
    nh  =  nh_in;
}

template <class Point>
cv::Point3f Table<Point>::table_cloud_centre(sensor_msgs::PointCloud2 msg, bool store_point, std::string collection) {

    cv::Point3f tcentre(0.0,0.0,0.0);

    //convert to pcl format
    cloud_ptr cc(new cloud_type());
    pcl::fromROSMsg(msg, *cc);

    //average
    float amount = cc->points.size();
    for(int i=0;i<amount;i++){
        tcentre.x += cc->points.at(i).x;
        tcentre.y += cc->points.at(i).y;
        tcentre.z += cc->points.at(i).z;
    }
    tcentre.x = tcentre.x/amount;
    tcentre.y = tcentre.y/amount;
    tcentre.z = tcentre.z/amount;

    //convert to a ros msg and store
    if(store_point){
        mongodb_store::MessageStoreProxy table_centre(*nh, collection);
        geometry_msgs::Point32 pp;
        pp.x = tcentre.x;
        pp.y = tcentre.y;
        pp.z = tcentre.z;
        table_centre.insert(pp);
    }
    return tcentre;
}

/*
cv::Point3f Table_Tool::table_centre(strands_perception_msgs::Table msg) {
    cv::Point3f tcentre;
    for(int j=0;j<result_tables[i]->tabletop.points.size();j++){
        tcentre.x += msg.tabletop.points.at(j).x;
        tcentre.y += msg.tabletop.points.at(j).y;
        tcentre.z += msg.tabletop.points.at(j).z;
    }

    float amount = msg.tabletop.points.size();
    tcentre.x = tcentre.x/amount;
    tcentre.y = tcentre.y/amount;
    tcentre.z = tcentre.z/amount;

    return tcentre;
}
 */

template <class Point>
void Table<Point>::dbtable_kdtree(std::string collection, std::vector<cv::Point3f>& table_centre_index) {
    //load table shapes
    mongodb_store::MessageStoreProxy table_merge(*nh, collection);
    std::vector< boost::shared_ptr<strands_perception_msgs::Table> > result_tables;
    table_merge.query<strands_perception_msgs::Table>(result_tables);

    /*
    double avg_p[3];
    std::vector<cv::Point2f> table_center_index;
    cv::Point2f center_2d;

    //calculate table centre
    avg_p[0] = 0;
    avg_p[1] = 0;
    avg_p[2] = 0;
    for(int i=0;i<result_tables.size();i++){
        for(int j=0;j<result_tables[i]->tabletop.points.size();j++){
            avg_p[0]  += result_tables[i]->tabletop.points.at(j).x;
            avg_p[1]  += result_tables[i]->tabletop.points.at(j).y;
            avg_p[2]  += result_tables[i]->tabletop.points.at(j).z;
        }

        if(Debug) {
            std::cout << avg_p[0] << "," << avg_p[1] << "," << avg_p[2] << std::endl;
        }

        avg_p[0] = avg_p[0]/result_tables[i]->tabletop.points.size();
        avg_p[1] = avg_p[1]/result_tables[i]->tabletop.points.size();
        avg_p[2] = avg_p[2]/result_tables[i]->tabletop.points.size();

        if(Debug) {
            std::cout << avg_p[0] << "," << avg_p[1] << "," << avg_p[2] << std::endl;

        }

        //push to index
        center_2d.x = avg_p[0];
        center_2d.y = avg_p[1];
        table_center_index.push_back(center_2d);

    }

    if(Debug) {
        ROS_INFO("table centre number: %lu", table_center_index.size());
        //publish and view once
        VIZ_Points vv(nh,"/table_centers");
        vv.pub_points(table_center_index);
    }

    //construct kdtree from index
    cv::flann::KDTreeIndexParams index_params;
    cv::flann::Index kdtree(cv::Mat(table_center_index).reshape(1),index_params);
    std::vector<float> query;
    std::vector<int> indices;
    std::vector<float> dists;

    for(int i=0;i<table_center_index.size();i++){
        query.push_back(table_center_index[i].x);
        query.push_back(table_center_index[i].y);
        //don't need param
        kdtree.knnSearch(query,indices,dists,table_knn);

        //check merge-able
        bool merge_able;
        merge_able=false;

        //TODO:(be careful)indices has at least the size of knn, if no neighbour just set the index 0
        if(indices[1]!=0){
            //delete that neighbour
            if(!merge_able){
                //indices[1]

            }
        }
    }
     */

}
