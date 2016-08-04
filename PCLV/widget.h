#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include "registration1.h"
#include "construction.h"
#include "gsurface.h"

namespace Ui {
class Widget;
}

class Widget : public QWidget
{
    Q_OBJECT

public:
    explicit Widget(QWidget *parent = 0);
    ~Widget();
    void refresh();

    pcl::visualization::PCLVisualizer *viz;
    pcl::visualization::PCLVisualizer *viz2;
    pcl::visualization::PCLVisualizer *viz3;

private:
    Ui::Widget *ui;

    void save_Cloud(CloudXYZRGBA::Ptr &cloud);
    //    bool IsRegistration;
    bool getcloud_btn_isfirst;
    //    void load_CloudLeft();
    bool oc_btn;



private slots:
    void on_refresh_btn_clicked();
    void on_ograbber_btn_clicked();
    void on_getcloud_btn_clicked();
    void on_surface_btn_clicked();
    void on_pushButton_clicked();

    void refreshviz(CloudXYZRGBA::Ptr ,std::string*);
    void refreshviz2(CloudXYZRGBA::Ptr );
    void refreshviz3(CloudNormal::Ptr ,bool );
    void setState(bool );
    void on_pushButton_2_clicked();
};

#endif // WIDGET_H




