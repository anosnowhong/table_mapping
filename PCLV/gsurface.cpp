#include "gsurface.h"
#include "ui_gsurface.h"

Gsurface::Gsurface(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Gsurface)
{
    ui->setupUi(this);
}

Gsurface::~Gsurface()
{
    delete ui;
}

void Gsurface::GreedyProcess(CloudXYZRGBA::Ptr cloud_in)
{
    pcl::io::savePCDFileBinary("./model.pcd",*cloud_in);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("./model.pcd",*cloud);
    //=====
    //pcl::NormalEstimation<PointT,pcl::Normal> n;
    pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    //pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    //tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(20);
    n.compute(*normals);

    CloudNormal::Ptr normal_cloud(new CloudNormal);
    pcl::concatenateFields(*cloud,*normals,*normal_cloud);//merge two types of Fields into one
    pcl::search::KdTree<PointNormalT>::Ptr tree2(new pcl::search::KdTree<PointNormalT>);
    //tree2->setInputCloud(normal_cloud);
    pcl::GreedyProjectionTriangulation<PointNormalT> gp3;
    pcl::PolygonMesh triangles;
    gp3.setSearchMethod(tree2);
    gp3.setSearchRadius(0.025);
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI/4);//45 degrees
    gp3.setMinimumAngle(M_PI/18);//10 dg
    gp3.setMaximumAngle(2*M_PI/3);//120 dg
    gp3.setNormalConsistency(false);
    gp3.setInputCloud(normal_cloud);

    gp3.reconstruct(triangles);

    pcl::visualization::PCLVisualizer *viewer(new pcl::visualization::PCLVisualizer("",false));
    ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());

    if(!viewer->updatePolygonMesh(triangles,"model"))
        viewer->addPolygonMesh(triangles,"model");

    ui->qvtkWidget->update();


}
