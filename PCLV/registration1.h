#ifndef REGISTRATION1_H
#define REGISTRATION1_H

#include <Qt>
#include <QMetaType>
#include <QThread>
//************vtk
#include <vtkRenderWindow.h>
//*************pcl
//===========cloud type
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//===========filter
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
//===========search
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/flann.h>
//===========registration
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
//===========Normal
#include <pcl/features/normal_3d.h>
//===========IO
#include <pcl/io/pcd_io.h>
//===========OpenNI
#include "openni_grabber1.h"
//=========PointRepresentation
#include "normal_pointrepresentation.h"
using pcl::visualization::PointCloudColorHandlerCustom;
using pcl::visualization::PointCloudColorHandlerGenericField;

//extern openni_grabber1 *v;
class registration1 : public QThread
{
    Q_OBJECT
public:
    registration1();
    void initial();
    void loadCloud();
    void run();

    CloudXYZRGBA::ConstPtr cloudin_1;
    CloudXYZRGBA::ConstPtr cloudin_2;
    CloudXYZRGBA::Ptr Final_Cloud;
    Eigen::Matrix4f GlobalTransform, pairtransform;
    int record_cloud;//count for registration
    bool IsRegistration;//for save function to check where the save cloud from?
    QString TransformMatrix;
    CloudXYZRGBA::Ptr fullcloud;

private:
signals:
    void compelePersent(int );
    void updateviz(CloudXYZRGBA::Ptr cloud_v,std::string*);
    void updateviz2(CloudXYZRGBA::Ptr cloud_v2);
    void updateviz3(CloudNormal::Ptr cloud_v3,bool isfirst);

};

#endif // REGISTRATION1_H
