#ifndef OPENNI_GRABBER1_H
#define OPENNI_GRABBER1_H

#include <Qt>
#include <QThread>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/openni_grabber.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> CloudXYZRGBA;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> CloudNormal;

class openni_grabber1 : public QThread
{
    Q_OBJECT
public:
    openni_grabber1();

    void cloudCallback(const CloudXYZRGBA::ConstPtr &cloud);
    void set_once(bool state);
    void set_state(bool state);

    pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud_;//for pass cloud we get from Grabber
    boost::mutex cloud_mutex_;

private:
    void run();
    bool check_state;
    bool check_once;//just for print out to see if we can go on other operation

signals:
     void grabber_state(bool);
};

#endif // OPENNI_GRABBER1_H
