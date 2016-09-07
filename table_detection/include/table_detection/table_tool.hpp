#include "table_detection/table_tool.h"
#include <table_detection/table_merge_info.h>

template <class Point>
Table<Point>::Table(){

}

template <class Point>
Table<Point>::Table(ros::NodeHandlePtr nh_in) {
    nh  =  nh_in;
}

template <class Point>
void Table<Point>::table_normal() {

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
void Table<Point>::dbtable_cloud_kdtree(std::string collection, pcl::KdTreeFLANN<point_type> &kdtree, int begin, int end) {
    //load table cloud (plane that extracted form single scan)
    mongodb_store::MessageStoreProxy table(*nh, collection);
    std::vector<boost::shared_ptr<sensor_msgs::PointCloud2> > result_tables;
    table.query<sensor_msgs::PointCloud2>(result_tables);

    //clouds sum
    cloud_ptr all_table_clouds(new cloud_type());
    cloud_ptr table_clouds(new cloud_type());
    if(end> result_tables.size()){
        ROS_INFO("index exceed range");
        return;
    }
    for (; begin < end; begin++) {
        pcl::fromROSMsg(*result_tables[begin], *table_clouds);
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

template <class Point>
void Table<Point>::table_merge(std::vector<std::vector<int> > merge_index){

    ROS_INFO("Merging tables...");
    mongodb_store::MessageStoreProxy table(*nh, "table_convex");
    std::vector<boost::shared_ptr<sensor_msgs::PointCloud2> > result_convex;
    table.query<sensor_msgs::PointCloud2>(result_convex);

    cloud_ptr cloud_sum(new cloud_type());
    cloud_ptr cloud_store(new cloud_type());
    cloud_ptr cloud_hull(new cloud_type());

    for (int i = 0; i < merge_index.size(); i++) {

        for(int j=0;j<merge_index[i].size();j++){
            pcl::fromROSMsg(*result_convex[merge_index[i][j]], *cloud_store);
            *cloud_sum += *cloud_store;
        }

        pcl::ConvexHull<Point> chull;
        chull.setDimension(2);
        chull.setInputCloud (cloud_sum);
        chull.reconstruct (*cloud_hull);

        sensor_msgs::PointCloud2 final_table;
        pcl::toROSMsg(*cloud_hull, final_table);
        final_table.header.frame_id =  "/map";

        mongodb_store::MessageStoreProxy table_final(*nh, "final_table");
        table_final.insert(final_table);

        //clear
        cloud_sum.reset(new cloud_type());
    }


}

/*
 * only compare overlap in one round
 */
template <class Point>
void Table<Point>::overlap_detect(std::string collection, std::vector<std::vector<int> > &overlap_index) {

    mongodb_store::MessageStoreProxy table(*nh, collection);
    std::vector<boost::shared_ptr<table_detection::table_neighbour_arr> > result_tables;
    table.query<table_detection::table_neighbour_arr>(result_tables);

    //int rounds_num = result_tables.size();
    //force on first round
    int rounds_num = 1;
    int startf = 0;

    /*
    if(result_tables.size()==1){
        int startf = 0;
    }
    else{
        //the former round info
        int startf = result_tables[rounds_num-2]->neighbour_arr.size();
    }
     */

    sensor_msgs::PointCloud2 convex_info;
    cloud_ptr convex_cloud(new cloud_type());
    sensor_msgs::PointCloud2 test_info;
    cloud_ptr test_cloud(new cloud_type());

    std::vector<float> verty;
    std::vector<float> vertx;
    std::vector<float> testy;
    std::vector<float> testx;

    std::vector<int> sub_overlap;
    int overlap=0;

    ROS_INFO("round size: %lu",result_tables[0]->neighbour_arr.size());
    for(;startf<result_tables[rounds_num-1]->neighbour_arr.size();startf++){
        convex_cloud.reset(new cloud_type());
        convex_info = (*result_tables[rounds_num-1]).neighbour_arr[startf].convex_cloud;
        pcl::fromROSMsg(convex_info,*convex_cloud);

        for(int pp=0;pp<convex_cloud->size();pp++){
            vertx.push_back(convex_cloud->at(pp).x);
            verty.push_back(convex_cloud->at(pp).y);
            //std::cout<<convex_cloud->at(pp).x<<std::endl;
        }
        //std::cout<<"vertices size: ======"<<vertx.size()<<std::endl;

        //111 startf=38 39 111-38=x 38+1< 111 38+2< 111 39+1< 111  110+1 < 111 false
        for(int increase=1;(startf+increase)<result_tables[rounds_num-1]->neighbour_arr.size();increase++){
            test_cloud.reset(new cloud_type());
            test_info = result_tables[rounds_num-1]->neighbour_arr[startf+increase].convex_cloud;
            pcl::fromROSMsg(test_info,*test_cloud);
            //std::cout<<"test cloud size: "<<test_cloud->size()<<std::endl;
            //TODO::if without cout not excute???
            for(int pt=0;pt<test_cloud->size();pt++){
                testx.push_back(test_cloud->at(pt).x);
                testy.push_back(test_cloud->at(pt).y);
                //std::cout<<"testx "<<test_cloud->at(pt).x<<std::endl;
            }
            //std::cout<<"testpoint size: ======"<<testx.size()<<std::endl;

            int i, j=0;
            for(int test_num=0;test_num < testx.size();test_num++){
                for (i = 0, j = vertx.size()-1; i < vertx.size(); j = i++) {
                    //ensure point height.y is between these to vertices
                    //std::cout<<"edge:"<<verty[i]<<","<<verty[j]<<std::endl;
                    //std::cout<<"testy"<<testy[test_num]<<std::endl;
                    if  ((verty[i]>testy[test_num]) != (verty[j]>testy[test_num])) {
                        //judge if on the left side or right side, if larger that testx ,testx is on the left side of edge
                        //std::cout<<"between edge:"<<verty[i]<<","<<verty[j]<<std::endl;
                        if((testx[test_num] < (vertx[j]-vertx[i]) * (testy[test_num]-verty[i]) / (verty[j]-verty[i]) + vertx[i]) ){
                            overlap = !overlap;
                            //std::cout<<"test left"<<std::endl;
                        }
                    }
                }

                //std::cout<<"test point: "<<testx[test_num]<<","<<testy[test_num]<<overlap<<std::endl;
                //std::cout<<"overlap: "<<overlap<<std::endl;
                //if one point is detect in another polygon
                if(overlap){
                    break;
                }
            }


            if(overlap){
                //index of the test case
                sub_overlap.push_back(startf+increase);
                //std::cout<<" sub "<<startf<<" ";
            }

            testx.clear();
            testy.clear();
            overlap = false;
        }
        //push the polygen index that overlap with startf
        //sub_overlap is 0 when no overlap detected
        if(sub_overlap.size()>0){
            //remember push itself last
            sub_overlap.push_back(startf);
            overlap_index.push_back(sub_overlap);
            //std::cout<<" end "<<startf<<" "<<std::endl;
        }

        vertx.clear();
        verty.clear();
        sub_overlap.clear();
    }


    //classify table that can be merged

    //sort
    for(int i=0;i<overlap_index.size();i++){
        std::sort(overlap_index[i].begin(),overlap_index[i].end());
    }

    //debug
    /*
    for(int i=0;i<overlap_index.size();i++){
        for(int j=0;j<overlap_index[i].size();j++){
            std::cout<<" "<<overlap_index[i][j];
        }
        std::cout<<std::endl;
    }
     */

    std::vector<int> v_intersection;
    std::vector<int> dest1;
    //have instersection?
    bool stillhave;
    do {
        stillhave=false;
        for (int inter=0; inter < overlap_index.size(); inter++) {
            for (int ins = 1; (inter + ins) < overlap_index.size(); ins++) {

                std::set_intersection(overlap_index[inter].begin(), overlap_index[inter].end(),
                                      overlap_index[inter + ins].begin(), overlap_index[inter + ins].end(),
                                      std::back_inserter(v_intersection));
                //then get union
                if (v_intersection.size() > 0) {
                    std::set_union(overlap_index[inter].begin(), overlap_index[inter].end(),
                                   overlap_index[inter + ins].begin(), overlap_index[inter + ins].end(),
                                   std::back_inserter(dest1));
                    //replace
                    overlap_index[inter] = dest1;
                    dest1.clear();
                    //delete
                    overlap_index.erase(overlap_index.begin() + inter + ins);
                    stillhave = true;
                }
                v_intersection.clear();

            }

            inter++;
        }

    }while (stillhave);

    std::cout<<overlap_index.size()<<std::endl;

    //debug
    mongodb_store::MessageStoreProxy merge(*nh, "merge_info");
    table_detection::table_merge_info merge_info;
    for(int i=0;i<overlap_index.size();i++){
        for(int j=0;j<overlap_index[i].size();j++){
            std_msgs::Int32 dd;
            dd.data = overlap_index[i][j];
            merge_info.merge_group.push_back(dd);
        }
        merge.insert(merge_info);
        merge_info.merge_group.clear();
    }
    ROS_INFO("merge done!");

    for(int i=0;i<overlap_index.size();i++){
        for(int j=0;j<overlap_index[i].size();j++){
            std::cout<<" "<<overlap_index[i][j];
        }
        std::cout<<std::endl;
    }
}

template <class Point>
void Table<Point>::merge_table_centre(std::string collection){
    mongodb_store::MessageStoreProxy table(*nh, collection);
    std::vector<boost::shared_ptr<geometry_msgs::Polygon> > result_tables;
    table.query<geometry_msgs::Polygon>(result_tables);

    int round = result_tables.size();

    point_type cc;
    cloud_ptr ccl(new cloud_type());

    //at least 2 round
    if(round==1) {
        return;
    }

    //construct point cloud form table centre
    for(int i=0;i<result_tables.size();i++){
        for(int j=0;j<result_tables[i]->points.size();j++){
            cc.x = result_tables[i]->points.at(j).x;
            cc.y = result_tables[i]->points.at(j).y;
            cc.z = result_tables[i]->points.at(j).z;
            ccl->push_back(cc);
        }
    }

    //query one nearest neighbour
    pcl::KdTreeFLANN<point_type >  kdtree;
    std::vector<int> q_ind;
    std::vector<float> q_dis;
    kdtree.setInputCloud(ccl);
    for(int i=0;i<result_tables.size();i++){
        for(int j=0;j<result_tables[i]->points.size();j++){

            cc.x = result_tables[i]->points.at(j).x;
            cc.y = result_tables[i]->points.at(j).y;
            cc.z = result_tables[i]->points.at(j).z;
            int f_num = kdtree.radiusSearch(cc,0.1,q_ind,q_dis);
            if(f_num!=0){
                //TODO:
            }
            q_ind.clear();
            q_dis.clear();
        }

    }

}
