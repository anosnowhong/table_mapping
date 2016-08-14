#include "registration_operator.h"

template <typename Point>
void registration_operator<Point>::trans_to_matrix(geometry_msgs::Vector3 vec)
{
    Eigen::MatrixXd m(4,4);
    m(0,3) = vec.x;
    m(1,3) = vec.y;
    m(2,3) = vec.z;
    m(0,0)=m(1,1)=m(2,2)=m(3,3) = 1;
    return m;
}

template <typename Point>
void registration_operator<Point>::rot_to_matrix(geometry_msgs::Quaternion qua)
{
    Eigen::MatrixXd m(4,4);
    //quaternion to matrix

    return m;
}
