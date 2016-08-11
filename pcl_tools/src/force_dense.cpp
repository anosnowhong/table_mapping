#include "pcl_all.h"

int main(int argc, char** argv)
{

    std::vector<int> filenames;
    C_xyz::Ptr tmp_cloud(new C_xyz());
    C_xyz::Ptr tmp_cloud2(new C_xyz());
    std::vector<int> ind;

    filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");

    std::cout<<"Found "<<filenames.size()<<" Files, new cloud will be generate under same folder..."<<std::endl;

    if(filenames.size() != 0)
    {
        for(int i=0;i<filenames.size();i++)
        {
            //loading pcd files
            if (pcl::io::loadPCDFile(argv[filenames[i]], *tmp_cloud)<0)
            {
                PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
                return -1;
            }
            //remove nan

            std::cout<<"...Converting, is dense? "<<tmp_cloud->is_dense<<argv[filenames[i]]<<std::endl;

            //pcl::removeNaNFromPointCloud(*tmp_cloud,*tmp_cloud2,ind);

			tmp_cloud->is_dense=1;
            std::cout<<"modify attirbute is_dense, force to dense: "<<tmp_cloud->is_dense<<argv[filenames[i]]<<std::endl;

            //save no nan point cloud
            std::string save_name = argv[filenames[i]];
            save_name.insert(save_name.length()-4, "_without_nan");

            pcl::io::savePCDFileASCII(save_name, *tmp_cloud2);

        }

    }
    else
        return -1;


    return 0;
}
