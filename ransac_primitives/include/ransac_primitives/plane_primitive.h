#ifndef PLANE_PRIMITIVE_H
#define PLANE_PRIMITIVE_H

#include "base_primitive.h"

#include <Eigen/Dense>

class plane_primitive : public base_primitive
{
public:
    // primitive shape parameters, with fitted smallest enclosing box of data:
    Eigen::Vector4d p; // the plane equation parameters
    Eigen::Vector3d c; // center of plane, also midpoint in fitted rectangle
    Eigen::Matrix<double, 3, 2> basis; // arbitrary basis orthogonal to normal
    Eigen::Vector2d sizes; // rough length of the sides of a fitted rectangle along basis

    // other parameters:
    Eigen::Quaterniond quat; // orientation of plane via rot matrix [p(1:3) basis]
    // 2d convex hull in basis, backprojected into 3d
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > convex_hull;
protected:
    void find_smallest_enclosing_box(Eigen::Vector2d& cmin, Eigen::Matrix2d& axes,
                                     Eigen::Vector2d& lengths, std::vector<cv::Point>& pts);
public:
    void switch_direction();
    void merge_planes(plane_primitive& other1, plane_primitive& other2);
    bool construct(const Eigen::MatrixXd& points, const Eigen::MatrixXd& normals,
                   double inlier_threshold, double angle_threshold);
    void compute_inliers(std::vector<int>& inliers, const Eigen::MatrixXd& points, const Eigen::MatrixXd& normals,
                         const std::vector<int>& inds, double inlier_threshold, double angle_threshold);
    void largest_connected_component(std::vector<int>& inliers, const Eigen::MatrixXd& points);
    int points_required();
    double distance_to_pt(const Eigen::Vector3d& pt);
    void direction_and_center(Eigen::Vector3d& direction, Eigen::Vector3d& center);
    double shape_size();
    void shape_data(Eigen::VectorXd& data);
    void shape_points(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& points);
    void compute_shape_size(const Eigen::MatrixXd& points);
    shape get_shape();
    void draw(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);
    base_primitive* instantiate();
    plane_primitive();
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif // PLANE_PRIMITIVE_H
