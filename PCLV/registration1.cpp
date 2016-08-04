#include "registration1.h"

registration1::registration1():
    Final_Cloud(new CloudXYZRGBA)
{
    initial();
    qRegisterMetaType<CloudXYZRGBA::Ptr> ("CloudXYZRGBA::Ptr");
    qRegisterMetaType<CloudNormal::Ptr> ("CloudNormal::Ptr");
}

void registration1::initial()
{
    cloudin_1.reset(new CloudXYZRGBA);
    cloudin_2.reset(new CloudXYZRGBA);
    fullcloud.reset(new CloudXYZRGBA);
    GlobalTransform=Eigen::Matrix4f::Identity();
    record_cloud=0;
    TransformMatrix="";
}

void registration1::run()
{
    CloudXYZRGBA::Ptr cloud1(new CloudXYZRGBA);
    CloudXYZRGBA::Ptr cloud2(new CloudXYZRGBA);
    CloudXYZRGBA::Ptr cloud_out(new CloudXYZRGBA);

    std::cerr<<"VoxelGrid"<<std::endl;
    pcl::VoxelGrid<PointT> vox;
    vox.setLeafSize(0.01,0.01,0.01);
    vox.setInputCloud(cloudin_1);//use the same cloud to cover itself
    vox.filter(*cloud1);
    vox.setInputCloud(cloudin_2);
    vox.filter(*cloud2);

    std::cerr<<"PassThrough"<<std::endl;
    pcl::PassThrough<PointT> pass;
    pass.setFilterFieldName("z");
    pass.setFilterLimits(1.0,2.5);
    pass.setInputCloud(cloud1);
    pass.filter(*cloud1);
    pass.setInputCloud(cloud2);
    pass.filter(*cloud2);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);

    std::cerr<<"ComputeNormal..."<<std::endl;
    CloudNormal::Ptr normal_cloud1(new CloudNormal);
    CloudNormal::Ptr normal_cloud2(new CloudNormal);
    pcl::NormalEstimation<PointT,PointNormalT> norm_est;

    norm_est.setSearchMethod(tree);
    norm_est.setKSearch(20);
    norm_est.setInputCloud(cloud1);
    norm_est.compute(*normal_cloud1);
    pcl::copyPointCloud(*cloud1,*normal_cloud1);
    norm_est.setInputCloud(cloud2);
    norm_est.compute(*normal_cloud2);
    pcl::copyPointCloud(*cloud2,*normal_cloud2);

    //***********
    normal_PointRepresentation point_representation;
    float alpha[4]  = {1.0,1.0,1.0,1.0};
    point_representation.setRescaleValues(alpha);//nothing have changed?

    //icp nonlinear
    std::cerr<<"ICP Start Setting..."<<std::endl;
    pcl::IterativeClosestPointNonLinear<PointNormalT,PointNormalT> reg;
    reg.setTransformationEpsilon(1e-6);
    reg.setMaxCorrespondenceDistance(0.1);

    reg.setPointRepresentation(boost::make_shared<const normal_PointRepresentation>(point_representation));

    reg.setInputSource(normal_cloud1);
    reg.setInputTarget(normal_cloud2);
    /*
      reg.setEuclideanFitnessEpsilon();
      reg.setMaximumIterations();
      reg.setCorrespondenceEstimation();
      */
    //add pointcloud to v3 before dynamic transform
    //PointCloudColorHandlerGenericField<PointNormalT> colornormal2(normal_cloud2,"colo");
    //if(!colornormal2.isCapable())
    //   PCL_WARN("Cannot create color handler");

    emit updateviz3(normal_cloud2,1);

    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
    std::cerr<<"ICP Start..."<<std::endl;
    CloudNormal::Ptr reg_result = normal_cloud1;
    //start iteration times
    reg.setMaximumIterations(2);
    for(int i=0;i<30;++i)
    {

        //std::cerr<<" Start..."<<i<<std::endl;
        normal_cloud1=reg_result;
        reg.setInputSource(normal_cloud1);
        reg.align(*reg_result);
        Ti = reg.getFinalTransformation()*Ti;

        //if the latest incremental variable minus previous is smaller than epsilon
        if(fabs((reg.getLastIncrementalTransformation()-prev).sum())<reg.getTransformationEpsilon())
            reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance()-0.001);
        prev = reg.getLastIncrementalTransformation();
        //std::cerr<<"            End..."<<i<<std::endl;
        //Refresh PointCloud

        emit compelePersent(i+1);

        emit updateviz3(normal_cloud1,0);
    }
    targetToSource = Ti.inverse();//inverse Matrix

    // final_transform=targetToSource;//recorde;

    //use targetToSource transform cloud2 and give the result to Final_cloud
    pcl::transformPointCloud(*cloud2,*cloud_out,targetToSource);

    *cloud_out+=*cloud1;//add two point cloud


    std::cerr<<"ICP Compeleted..."<<std::endl;


    //输出两两变换的矩阵
    //TransformMatrix+=tt;
    TransformMatrix+="\n";
    TransformMatrix+="===================";
    TransformMatrix+="\n";
    //ui->textEdit->setText(TransformMatrix);
    //std::cout<<TransformMatrix;

    //Done!

    std::cerr<<"registration compeleted"<<std::endl;

    pcl::transformPointCloud(*cloud_out,*Final_Cloud,GlobalTransform);
    GlobalTransform=targetToSource*GlobalTransform;//cumulated...transform
    //save

    std::stringstream ss;
    ss<<"reg"<<record_cloud<<".pcd";
    pcl::io::savePCDFile(ss.str(),*Final_Cloud,true);
    //pcl::io::savePCDFile(ss.str(),*Final_Cloud,false);
    record_cloud++;

    //exchange
    cloudin_1=cloudin_2;

    //viz1 load pcd file
    loadCloud();

    quit();
}

void registration1::loadCloud()
{
    for(int a=0;a<record_cloud;a++)
    {
        CloudXYZRGBA::Ptr cloudfile(new CloudXYZRGBA);
        std::stringstream file;
        file<<"reg"<<a<<".pcd";
        pcl::io::loadPCDFile(file.str(),*cloudfile);
        *fullcloud+=*cloudfile;//add to one cloud

        std::stringstream cloud_number;
        cloud_number<<a;
        std::string *cloud_name=new(std::string);
        *cloud_name="cloud_name"+cloud_number.str();

        std::cout<<a<<std::endl;

        file.str("");
        emit updateviz(cloudfile,cloud_name);

    }
}
