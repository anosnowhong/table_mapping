#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZ P_xyz;
typedef pcl::PointCloud<P_xyz> C_xyz;
typedef C_xyz::CloudVectorType CV_xyz;

