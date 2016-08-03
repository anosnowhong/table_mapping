#ifndef CYLINDER_PRIMITIVE_H
#define CYLINDER_PRIMITIVE_H

#include "base_primitive.h"

#include <Eigen/Dense>

class cylinder_primitive : public base_primitive
{
public:
    // primitive shape parameters:
    Eigen::Vector3d a; // main axis of cylinder
    Eigen::Vector3d c; // center of cylinder, also along axis
    double r; // radius of cylinder
    double h; // height of cylinder

    // other parameters:
    Eigen::Matrix<double, 3, 2> basis; // arbitrary basis orthogonal to a
    double max_radius; // maximum allowed radius
    static int cylinders_drawn; // used to assign ID of cylinder
public:
    bool construct(const Eigen::MatrixXd& points, const Eigen::MatrixXd& normals,
                   double inlier_threshold, double angle_threshold);
    void compute_inliers(std::vector<int>& inliers, const Eigen::MatrixXd& points, const Eigen::MatrixXd& normals,
                         const std::vector<int>& inds, double inlier_threshold, double angle_threshold);
    void largest_connected_component(std::vector<int>& inliers, const Eigen::MatrixXd& points);
    int points_required();
    double distance_to_pt(const Eigen::Vector3d& pt);
    void direction_and_center(Eigen::Vector3d& direction, Eigen::Vector3d& center);
    void compute_shape_size(const Eigen::MatrixXd& points);
    double shape_size();
    void shape_data(Eigen::VectorXd& data);
    void shape_points(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& points);
    shape get_shape();
    void draw(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);
    base_primitive* instantiate();
    cylinder_primitive();
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif // CYLINDER_PRIMITIVE_H
