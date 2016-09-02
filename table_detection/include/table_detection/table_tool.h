//
//table points , structure, properties
//also contains some db operations
//

#include <strands_perception_msgs/Table.h>

#ifndef TABLE_MAPPING_TABLE_TOOL_H
#define TABLE_MAPPING_TABLE_TOOL_H

class Table_Tool{

public:
    Table_Tool();

    /* construct a kdtree to store table centre(which is calculate during each scan), (load all table msg from db)
     * Given db collection that contains strands_perception_msg
     * calculate all the centre in given collection, and put in a given vector
     */
    void dbtable_kdtree(std::string collection, std::vector<cv::Point3f>& table_centre_index);
    /*
     * calculate table centre for a single scan
     * store and collection is enabled by default ,
     * return cv::Point3f
     */
    cv::Point3f table_cloud_centre(strands_perception_msgs::Table msg,
                                   bool store_point=ture,
                                   std::string collection="table_centre");

private:

};

cv::Point3f Table_Tool::table_cloud_centre(){

}

cv::Point3f Table_Tool::table_centre(strands_perception_msgs::Table msg) {
    cv::Point3f tcentre;
    for(int j=0;j<result_tables[i]->tabletop.points.size();j++){
        tcentre.x += msg.tabletop.points.at(j).x;
        tcentre.y += msg.tabletop.points.at(j).y;
        tcentre.z += msg.tabletop.points.at(j).z;
    }

    int amount = msg.tabletop.points.size();
    tcentre.x = tcentre.x/amount;
    tcentre.y = tcentre.y/amount;
    tcentre.z = tcentre.z/amount;

    return tcentre;
}

void Table_Tool::table_kdtree(std::string collection) {
    //load table shapes
    mongodb_store::MessageStoreProxy table_merge(*nh, collection);
    std::vector< boost::shared_ptr<strands_perception_msgs::Table> > result_tables;
    table_merge.query<strands_perception_msgs::Table>(result_tables);

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

}
#endif //TABLE_MAPPING_TABLE_TOOL_H
