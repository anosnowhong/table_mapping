#include "registration_operator.h"

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

    pcl::io::savePCDFileASCII("/home/parallels/debug/whole_cloud.pcd", *cloud_whole);
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

