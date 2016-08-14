#ifndef REGISTRATION_OPERATOR_H
#define REGISTRATION_OPERATOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_impl.h>
#include <Eigen/Dense>

template <typename Point>
class registration_operator {
public:
    typedef Point point_type;
    typedef pcl::PointCloud <point_type> cloud_type;
    typedef typename cloud_type::Ptr cloud_ptr;
    typedef typename cloud_type::ConstPtr cloud_const_ptr;

private:
    void trans_to_matrix(geometry_msgs::Vector3 vec);
    void rot_to_matrix(geometry_msgs::Quaternion qua);

};


#endif
