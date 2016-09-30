#include <table_detection/table_neighbour_arr.h>
#include <mongodb_store/message_store.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "data_change");
    ros::NodeHandle n;

    mongodb_store::MessageStoreProxy change(n,"table_centre_increment");
    std::vector< boost::shared_ptr<table_detection::table_neighbour_arr> > result;
    change.query<table_detection::table_neighbour_arr>(result);

    std::vector<std::vector<int> > last2round;
    std::vector<int> crr_round;

    //use size of the previous round
    int round_size = result[result.size()-2]->neighbour_arr.size();

    //compare the last two rounds
    for(int i=result.size()-2;i<result.size();i++){
        for(int j=0;j<round_size;j++){
            crr_round.push_back(result[i]->neighbour_arr[j].neighbour.data);
        }
        last2round.push_back(crr_round);
        crr_round.clear();
    }

    for(int i=0;i<round_size;i++){
        std::cout<<last2round[1][i]-last2round[0][i]<<" ";
    }
    std::cout<<std::endl;

}
