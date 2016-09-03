#include "table_detection/table_tool.h"

template <class Point>
Table<Point>::Table(){

}

template <class Point>
Table<Point>::Table(ros::NodeHandlePtr nh_in) {
    nh  =  nh_in;
}

template <class Point>
Point Table<Point>::table_cloud_centre(sensor_msgs::PointCloud2 msg, bool store_point, std::string collection) {

    point_type tcentre;
    tcentre.x = tcentre.y = tcentre.z = 0.0;

    //convert to pcl format
    cloud_type cc;
    pcl::fromROSMsg(msg, cc);

    //average
    float amount = cc.points.size();
    for(int i=0;i<amount;i++){
        tcentre.x += cc.points.at(i).x;
        tcentre.y += cc.points.at(i).y;
        tcentre.z += cc.points.at(i).z;
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
void Table<Point>::dbtable_cloud_centre(std::string collection, std::vector<point_type>& table_centre_index) {
    //load table cloud (plane that extracted form single scan)
    mongodb_store::MessageStoreProxy table(*nh, collection);
    std::vector<boost::shared_ptr<sensor_msgs::PointCloud2> > result_tables;
    table.query<sensor_msgs::PointCloud2>(result_tables);
    //convert msg to cloud
    std::vector<cloud_type> table_clouds;
    table_clouds.resize(result_tables.size());
    for (int i = 0; i < result_tables.size(); i++) {
        pcl::fromROSMsg(*result_tables[i], table_clouds[i]);
    }

    //std::vector<cv::Point2f> table_center_index;
    //cv::Point2f center_2d;

    std::vector<point_type> table_center_index;
    point_type center_3d;

    //calculate table centre
    for (int i = 0; i < table_clouds.size(); i++) {
        for (int j = 0; j < table_clouds[i].width; j++) {
            center_3d.x += table_clouds[i].at(j).x;
            center_3d.y += table_clouds[i].at(j).y;
            center_3d.z += table_clouds[i].at(j).z;
        }


        center_3d.x = center_3d.x / table_clouds[i].width;
        center_3d.y = center_3d.y / table_clouds[i].width;
        center_3d.z = center_3d.z / table_clouds[i].width;

        if (Debug) {
            std::cout << center_3d.x << "," << center_3d.y << "," << center_3d.z << std::endl;
        }

        table_center_index.push_back(center_3d);
    }
}

template <class Point>
void Table<Point>::dbtable_kdtree(std::string collection, pcl::KdTreeFLANN<point_type> &kdtree) {
    //load table cloud (plane that extracted form single scan)
    mongodb_store::MessageStoreProxy table(*nh, collection);
    std::vector<boost::shared_ptr<sensor_msgs::PointCloud2> > result_tables;
    table.query<sensor_msgs::PointCloud2>(result_tables);
    //convert msg to cloud
    std::vector<cloud_type > table_clouds;
    table_clouds.resize(result_tables.size());
    for (int i = 0; i < result_tables.size(); i++) {
        pcl::fromROSMsg(*result_tables[i], table_clouds[i]);
    }

    //construct point cloud from indices
    cloud_ptr table_centre_cloud(new cloud_type());
    point_type center_3d;

    //calculate table centre
    for (int i = 0; i < table_clouds.size(); i++) {
        for (int j = 0; j < table_clouds[i].width; j++) {
            center_3d.x += table_clouds[i].at(j).x;
            center_3d.y += table_clouds[i].at(j).y;
            center_3d.z += table_clouds[i].at(j).z;
        }

        center_3d.x = center_3d.x / table_clouds[i].width;
        center_3d.y = center_3d.y / table_clouds[i].width;
        center_3d.z = center_3d.z / table_clouds[i].width;

        if (Debug) {
            std::cout << center_3d.x << "," << center_3d.y << "," << center_3d.z << std::endl;
        }

        table_centre_cloud->push_back(center_3d);
    }

    //construct kdtree from index
    kdtree.setInputCloud(table_centre_cloud);
}

template <class Point>
void Table<Point>::dbtable_cloud_kdtree(std::string collection, pcl::KdTreeFLANN<point_type> &kdtree) {
    //load table cloud (plane that extracted form single scan)
    mongodb_store::MessageStoreProxy table(*nh, collection);
    std::vector<boost::shared_ptr<sensor_msgs::PointCloud2> > result_tables;
    table.query<sensor_msgs::PointCloud2>(result_tables);

    //clouds sum
    cloud_ptr all_table_clouds(new cloud_type());
    cloud_ptr table_clouds(new cloud_type());
    for (int i = 0; i < result_tables.size(); i++) {
        pcl::fromROSMsg(*result_tables[i], *table_clouds);
        *all_table_clouds += *table_clouds;
    }

    //construct kdtree from index
    kdtree.setInputCloud(all_table_clouds);
}
/*
    if (Debug) {
        ROS_INFO("table centre number: %lu", table_center_index.size());
        //publish and view once
        VIZ_Points vv(nh, "/table_centers");
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
