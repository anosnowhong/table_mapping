#ifndef REGISTRATION_OPERATOR_H
#define REGISTRATION_OPERATOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/flann.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>

#include <Eigen/Dense>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

template <typename Point>
class registration_operator {
public:
    typedef Point point_type;
    typedef pcl::PointCloud <point_type> cloud_type;
    typedef typename cloud_type::Ptr cloud_ptr;
    typedef typename cloud_type::ConstPtr cloud_const_ptr;

public:
    //
    registration_operator(cloud_ptr cloud1, cloud_ptr cloud2,
                          geometry_msgs::Vector3& vec,
                          geometry_msgs::Quaternion& qua);

    //tranform a point cloud to global coordinate using tf tree data(global is /map frame)
    registration_operator(cloud_ptr cloud_sample,
                          geometry_msgs::Vector3& vec,
                          geometry_msgs::Quaternion& qua);


    void registration();
    void to_global(cloud_type& output);
private:
    Eigen::MatrixXf trans_to_matrix(geometry_msgs::Vector3 vec);
    Eigen::MatrixXf rot_to_matrix(geometry_msgs::Quaternion qua);

    cloud_ptr cloud_source, cloud_target, cloud_whole;//two point cloud to stitch to a final whole cloud
    Eigen::MatrixXf ptu_transform;

};


#endif
