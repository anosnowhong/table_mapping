#ifndef NORMAL_POINTREPRESENTATION_H
#define NORMAL_POINTREPRESENTATION_H
#include <pcl/point_representation.h>

class normal_PointRepresentation : public pcl::PointRepresentation<pcl::PointNormal>
{
    using pcl::PointRepresentation<pcl::PointNormal>::nr_dimensions_;
public:
    normal_PointRepresentation();
    virtual void copyToFloatArray(const pcl::PointNormal &p, float *out) const;
};

#endif // NORMAL_POINTREPRESENTATION_H
