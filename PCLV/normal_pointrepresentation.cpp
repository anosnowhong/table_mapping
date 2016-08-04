#include "normal_pointrepresentation.h"

normal_PointRepresentation::normal_PointRepresentation()
{
    nr_dimensions_=4;
}

void normal_PointRepresentation::copyToFloatArray(const pcl::PointNormal &p,float *out) const
{
    out[0]=p.x;
    out[1]=p.y;
    out[2]=p.z;
    out[3]=p.curvature;
}
