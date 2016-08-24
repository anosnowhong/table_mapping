#include <ros/ros.h>
#include <mongodb_store/message_store.h>
#include <strands_perception_msgs/Table.h>
#include <visualization_msgs/MarkerArray.h>

//load all tables planes form mongodb and publish them
int main(int argc, char** argv)
{
    ros::init(argc, argv, "viz_table");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    std::string collection;
    std::vector<int> indices;
    pn.param<std::string>("collection",collection, "whole_tables");
    pn.getParam("specify_index",indices);

    ros::Publisher pub = n.advertise<visualization_msgs::MarkerArray>("table_planes", 1);

    mongodb_store::MessageStoreProxy table_store(n, collection);
    std::vector< boost::shared_ptr<strands_perception_msgs::Table> > result_tables;
    table_store.query<strands_perception_msgs::Table>(result_tables);

    visualization_msgs::MarkerArray mark_arr;
    //mark_arr.markers.resize(result_tables.size());
    mark_arr.markers.resize(indices.size());

    while(ros::ok())
    {
        //for(int i=0;i<result_tables.size();i++)
        for(int i=0;i<indices.size();i++)
        {
            //mark_arr.markers[i].header.frame_id=result_tables[i]->header.frame_id;
            mark_arr.markers[i].header.frame_id=result_tables[indices[i]]->header.frame_id;
            mark_arr.markers[i].id = i;
            mark_arr.markers[i].header.stamp = ros::Time();
            mark_arr.markers[i].ns = "tables";
            mark_arr.markers[i].type = visualization_msgs::Marker::LINE_STRIP;
            mark_arr.markers[i].action= visualization_msgs::Marker::ADD;

            //size_t n = result_tables[i]->tabletop.points.size();
            size_t n = result_tables[indices[i]]->tabletop.points.size();
            mark_arr.markers[i].points.resize(2*n);
            for (size_t j = 0; j < n-1 ; ++j) {
                /*
                mark_arr.markers[i].points[2*j+2].x = result_tables[i]->tabletop.points[j].x;
                mark_arr.markers[i].points[2*j+2].y = result_tables[i]->tabletop.points[j].y;
                mark_arr.markers[i].points[2*j+2].z = result_tables[i]->tabletop.points[j].z;

                mark_arr.markers[i].points[2*j+3].x = result_tables[i]->tabletop.points[j].x;
                mark_arr.markers[i].points[2*j+3].y = result_tables[i]->tabletop.points[j].y;
                mark_arr.markers[i].points[2*j+3].z = result_tables[i]->tabletop.points[j].z;
                 */
                mark_arr.markers[i].points[2*j+2].x = result_tables[indices[i]]->tabletop.points[j].x;
                mark_arr.markers[i].points[2*j+2].y = result_tables[indices[i]]->tabletop.points[j].y;
                mark_arr.markers[i].points[2*j+2].z = result_tables[indices[i]]->tabletop.points[j].z;

                mark_arr.markers[i].points[2*j+3].x = result_tables[indices[i]]->tabletop.points[j].x;
                mark_arr.markers[i].points[2*j+3].y = result_tables[indices[i]]->tabletop.points[j].y;
                mark_arr.markers[i].points[2*j+3].z = result_tables[indices[i]]->tabletop.points[j].z;
            }

            /*
            mark_arr.markers[i].points[0].x = result_tables[i]->tabletop.points[n-1].x;
            mark_arr.markers[i].points[0].y = result_tables[i]->tabletop.points[n-1].y;
            mark_arr.markers[i].points[0].z = result_tables[i]->tabletop.points[n-1].z;
            mark_arr.markers[i].points[1].x = result_tables[i]->tabletop.points[0].x;
            mark_arr.markers[i].points[1].y = result_tables[i]->tabletop.points[0].y;
            mark_arr.markers[i].points[1].z = result_tables[i]->tabletop.points[0].z;
             */
            mark_arr.markers[i].points[0].x = result_tables[indices[i]]->tabletop.points[n-1].x;
            mark_arr.markers[i].points[0].y = result_tables[indices[i]]->tabletop.points[n-1].y;
            mark_arr.markers[i].points[0].z = result_tables[indices[i]]->tabletop.points[n-1].z;
            mark_arr.markers[i].points[1].x = result_tables[indices[i]]->tabletop.points[0].x;
            mark_arr.markers[i].points[1].y = result_tables[indices[i]]->tabletop.points[0].y;
            mark_arr.markers[i].points[1].z = result_tables[indices[i]]->tabletop.points[0].z;

            mark_arr.markers[i].scale.x = 0.01;
            mark_arr.markers[i].scale.y = 0.01;
            mark_arr.markers[i].scale.z = 0.01;
            mark_arr.markers[i].pose.orientation.w = 1;
            mark_arr.markers[i].color.a = 1.0;
            mark_arr.markers[i].color.r = 1.0;
            mark_arr.markers[i].color.g = 0.0;
            mark_arr.markers[i].color.b = 0.0;
        }
        pub.publish(mark_arr);
        sleep(1);
    }
    ros::spin();
}

