#include "registration_operator.h"

template <typename Point>
registration_operator<Point>::registration_operator():
        cloud_source(new cloud_type()), cloud_target(new cloud_type()), cloud_whole(new cloud_type())
{

}
template <typename Point>
registration_operator<Point>::registration_operator(cloud_ptr cloud1, cloud_ptr cloud2, geometry_msgs::Vector3 &vec,
                                                    geometry_msgs::Quaternion &qua):
        cloud_source(new cloud_type()), cloud_target(new cloud_type()), cloud_whole(new cloud_type())
{
    //init registration procedure
    cloud_source = cloud1;
    cloud_target = cloud2;
    ptu_transform = trans_to_matrix(vec) * rot_to_matrix(qua);
    std::cout<<ptu_transform<<std::endl;
}


template <typename Point>
registration_operator<Point>::registration_operator(cloud_ptr cloud_sample, geometry_msgs::Vector3 &vec,
                                                    geometry_msgs::Quaternion &qua):
        cloud_source(new cloud_type()), cloud_target(new cloud_type()), cloud_whole(new cloud_type())
{
    cloud_source = cloud_sample;
    ptu_transform = trans_to_matrix(vec) * rot_to_matrix(qua);
}

template <typename Point>
void registration_operator<Point>::to_global(cloud_type& output) {
    pcl::transformPointCloud(*cloud_source,output,ptu_transform);
    //pcl::io::savePCDFileASCII("/home/parallels/debug/whole_cloud1.pcd", tmp_cloud);
}
template <typename Point>
void registration_operator<Point>::registration() {
    cloud_type tmp_cloud;
    pcl::transformPointCloud(*cloud_source,tmp_cloud,ptu_transform);
    *cloud_whole = *cloud_target + tmp_cloud;

    //pcl::io::savePCDFileASCII("/home/parallels/debug/whole_cloud.pcd", *cloud_whole);
}

/*
 *Traditional ICP algorithm,based on PTU movement make a more accurate transformation.
 */
template <typename Point>
Eigen::Matrix4f registration_operator<Point>::accurate_icp(cloud_ptr cloudin_1, cloud_ptr cloudin_2, cloud_ptr tfed_cloud){

    cloud_ptr cloud1(new cloud_type());
    cloud_ptr cloud2(new cloud_type());
    cloud_ptr cloud_out(new cloud_type());

    pcl::VoxelGrid<Point> vox;
    vox.setLeafSize(0.05,0.05,0.05);
    vox.setInputCloud(cloudin_1);//use the same cloud to cover itself
    vox.filter(*cloud1);
    vox.setInputCloud(cloudin_2);
    vox.filter(*cloud2);

    /*
    pcl::PassThrough<PointT> pass;
    pass.setFilterFieldName("z");
    pass.setFilterLimits(1.0,2.5);
    pass.setInputCloud(cloud1);
    pass.filter(*cloud1);
    pass.setInputCloud(cloud2);
    pass.filter(*cloud2);
     */

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    CloudNormal::Ptr normal_cloud1(new CloudNormal);
    CloudNormal::Ptr normal_cloud2(new CloudNormal);
    pcl::NormalEstimation<Point,PointNormalT> norm_est;

    norm_est.setSearchMethod(tree);
    norm_est.setKSearch(20);
    norm_est.setInputCloud(cloud1);
    norm_est.compute(*normal_cloud1);
    pcl::copyPointCloud(*cloud1,*normal_cloud1);
    norm_est.setInputCloud(cloud2);
    norm_est.compute(*normal_cloud2);
    pcl::copyPointCloud(*cloud2,*normal_cloud2);

    normal_PointRepresentation point_representation;
    float alpha[4]  = {1.0,1.0,1.0,1.0};
    point_representation.setRescaleValues(alpha);//nothing have changed?

    //icp nonlinear
    ROS_INFO("ICP Start Setting...");
    pcl::IterativeClosestPointNonLinear<PointNormalT,PointNormalT> reg;
    reg.setTransformationEpsilon(1e-6);
    reg.setMaxCorrespondenceDistance(0.1);

    reg.setPointRepresentation(boost::make_shared<const normal_PointRepresentation>(point_representation));

    reg.setInputSource(normal_cloud1);
    reg.setInputTarget(normal_cloud2);

    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;

    ROS_INFO("ICP Start...");
    CloudNormal::Ptr reg_result = normal_cloud1;
    //start iteration times
    reg.setMaximumIterations(2);
    for(int i=0;i<30;++i)
    {
        normal_cloud1=reg_result;
        reg.setInputSource(normal_cloud1);
        reg.align(*reg_result);
        Ti = reg.getFinalTransformation()*Ti;

        //if the latest incremental variable minus previous is smaller than epsilon
        if(fabs((reg.getLastIncrementalTransformation()-prev).sum())<reg.getTransformationEpsilon())
            reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance()-0.001);
        prev = reg.getLastIncrementalTransformation();

    }
    targetToSource = Ti.inverse();//inverse Matrix

    //use targetToSource transform cloud2 and give the result to Final_cloud
    pcl::transformPointCloud(*cloud2,*cloud_out,targetToSource);

    *cloud_out+=*cloud1;//add two point cloud

    //output cloud2 that registed to cloud1's coordinate
    *tfed_cloud = *cloud_out;
    ROS_INFO("ICP Compeleted...");

    return targetToSource;

    //std::cout<<"Matrix: "<<TransformMatrix<<std::endl;

}

template <typename Point>
Eigen::MatrixXf registration_operator<Point>::trans_to_matrix(geometry_msgs::Vector3 vec)
{
    Eigen::Matrix4f m=Eigen::Matrix4f::Identity();
    m(0,3) = vec.x;
    m(1,3) = vec.y;
    m(2,3) = vec.z;
    m(0,0)=m(1,1)=m(2,2)=m(3,3) = 1.0;
    return m;
}

template <typename Point>
Eigen::MatrixXf registration_operator<Point>::rot_to_matrix(geometry_msgs::Quaternion qua)
{
    //quaternion to matrix
    double a11 = 1 - 2 * pow(qua.y, 2) - 2 * pow(qua.z, 2);
    double a12 = 2 * qua.x * qua.y - 2 * qua.z * qua.w;
    double a13 = 2 * qua.x * qua.z + 2 * qua.y * qua.w;
    double a21 = 2 * qua.x * qua.y + 2 * qua.z * qua.w;
    double a22 = 1 - 2 * pow(qua.x, 2) - 2 * pow(qua.z, 2);
    double a23 = 2 * qua.y * qua.z - 2 * qua.x * qua.w;
    double a31 = 2 * qua.x * qua.z - 2 * qua.y * qua.w;
    double a32 = 2 * qua.y * qua.z + 2 * qua.x * qua.w;
    double a33 = 1 - 2 * pow(qua.x, 2) - 2 * pow(qua.y, 2);

    Eigen::MatrixXf m(4,4);
    m<<a11,a12,a13,0.0,
            a21,a22,a23,0.0,
            a31,a32,a33,0.0,
            0.0,0.0,0.0,1.0;
    return m;
}

