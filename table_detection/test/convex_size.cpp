#include <mongodb_store/message_store.h>
#include <table_detection/Table.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "db_cloud_extraction");
    ros::NodeHandlePtr nh;
    nh.reset(new ros::NodeHandle);
    ros::NodeHandle pn("~");

    mongodb_store::MessageStoreProxy dbcc(*nh, "final_table");
    std::vector<boost::shared_ptr<table_detection::Table> > result_tables;
    dbcc.query<table_detection::Table>(result_tables);

    std::vector<std::vector<double> > xx;
    std::vector<std::vector<double> > yy;

    std::vector<double>  x;
    std::vector<double>  y;

    for(int i=0;i<result_tables.size();i++){
        for(int j=0;j<result_tables[i]->tabletop.points.size();j++){

            //std::cout<<"=== "<<std::endl;
            x.push_back(result_tables[i]->tabletop.points.at(j).x);
            y.push_back(result_tables[i]->tabletop.points.at(j).y);
        }

        //the first point
        x.push_back(x[0]);
        y.push_back(y[0]);

        xx.push_back(x);
        yy.push_back(y);

        x.clear();
        y.clear();
    }


    for(int j=0;j<xx.size();j++){

        double area = 0;
        for( int i = 1; i <= xx[j].size(); i ++ ){
            //area += xx[j][i+1]*(yy[j][i+2]-yy[j][i]) + yy[j][i+1]*(xx[j][i]-xx[j][i+2]);
            area += (xx[j][i]*yy[j][i+1] - xx[j][i+1]*yy[0][i]);
            //std::cout<<area<<std::endl;
        }
        area /= 2;
        std::cout<<area<<std::endl;
        std::cout<<"=="<<std::endl;
    }

    return 0;
}
