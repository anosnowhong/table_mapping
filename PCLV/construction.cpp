#include "construction.h"

construction::construction()
{
}

void construction::GreedyProcess(CloudXYZRGBA::Ptr cloud)
{
        pcl::NormalEstimation<PointT,PointNormalT> n;
        CloudNormal::Ptr cnormals(new CloudNormal);
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);

        tree->setInputCloud(cloud);
        n.setInputCloud(cloud);
        n.setSearchMethod(tree);
        n.setKSearch(20);
        n.compute(*cnormals);

        pcl::search::KdTree<PointNormalT>::Ptr tree2(new pcl::search::KdTree<PointNormalT>);
        tree2->setInputCloud(cnormals);
        //Greedy Method
        pcl::GreedyProjectionTriangulation<PointNormalT> gp3;
        pcl::PolygonMesh triangles;//储存最后三角化的网格模型
        gp3.setSearchRadius(0.025);
        gp3.setMu(2.5);
        gp3.setMaximumNearestNeighbors(100);
        gp3.setMaximumSurfaceAngle(M_PI/4);
        gp3.setMinimumAngle(M_PI/18);
        gp3.setMaximumAngle(2*M_PI/3);
        //法线一致
        gp3.setNormalConsistency(false);

        gp3.setInputCloud(cnormals);

        gp3.setSearchMethod(tree2);
        gp3.reconstruct(triangles);

        std::vector<int> parts=gp3.getPartIDs();
        std::vector<int> states=gp3.getPointStates();
        pcl::visualization::PCLVisualizer *viewer(new pcl::visualization::PCLVisualizer("",false));

        //view in visualizer
       //viewer->addPolygonMesh(triangles,"my");
        //viewer->addCoordinateSystem(1.0);
        //viewer->initCameraParameters();


}
