#include "widget.h"
#include "ui_widget.h"
#include <QFileDialog>
#include <QDebug>

openni_grabber1 *v=new openni_grabber1();
registration1 Ris;
Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    viz=new pcl::visualization::PCLVisualizer("",false);
    viz2=new pcl::visualization::PCLVisualizer("",false);
    viz3=new pcl::visualization::PCLVisualizer("",false);
    ui->setupUi(this);
    //IsRegistration=false;
    getcloud_btn_isfirst=true;
    oc_btn=true;

    ui->qvtkWidget->SetRenderWindow(viz->getRenderWindow());
    ui->qvtkWidget_2->SetRenderWindow(viz2->getRenderWindow());
    ui->qvtkWidget_3->SetRenderWindow(viz3->getRenderWindow());
    //v->viz->setupInteractor(ui->qvtkWidget->GetInteractor(),ui->qvtkWidget->GetRenderWindow());
    ui->qvtkWidget->update();
    ui->qvtkWidget_2->update();
    ui->qvtkWidget_3->update();

    //check state
    connect(v,SIGNAL(grabber_state(bool)),this,SLOT(setState(bool)));
    //registration
    connect(&Ris,SIGNAL(compelePersent(int)),ui->progressBar,SLOT(setValue(int)));

    connect(&Ris,SIGNAL(updateviz(CloudXYZRGBA::Ptr,std::string*)),this,SLOT(refreshviz(CloudXYZRGBA::Ptr,std::string* )));
    //connect(&Ris,SIGNAL(updateviz2(CloudXYZRGBA::Ptr))),this,SLOT(refreshviz2(CloudXYZRGBA::Ptr));
    connect(&Ris,SIGNAL(updateviz3(CloudNormal::Ptr,bool)),this,SLOT(refreshviz3(CloudNormal::Ptr,bool)));

    std::cerr<<"Ready!"<<std::endl;
}

Widget::~Widget()
{
    delete ui;
}

void Widget::refresh()
{
    if(v->cloud_mutex_.try_lock())
    {
        std::cerr<<"widget refresh locked"<<std::endl;
        //here we only read, so may not need lock
        pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud_in = v->cloud_;
        //pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_in = v->cloud_;
        if(!cloud_in->empty())
        {
            if(!viz->updatePointCloud(cloud_in,"OpenNICloud"))
            {
                viz->addPointCloud(cloud_in,"OpenNICloud");
                viz->resetCameraViewpoint("OpenNICloud");
            }
        }
        v->cloud_mutex_.unlock();
        std::cerr<<"widget refresh unlocked"<<std::endl;
    }
    ui->qvtkWidget->GetRenderWindow()->Render();
    /*
    int elapsed_time = now.elapsed();
    int time = 30 -elapsed_time;

    std::cerr<<"elapsed_time: "<<elapsed_time<<std::endl;
    if(time<0)
        time = 0;
    QTimer::singleShot(time,this,SLOT(refresh()));
    */
}

void Widget::save_Cloud(CloudXYZRGBA::Ptr &cloud)
{
    /*
    if(IsRegistration)
    {
        std::stringstream ss;
        ss<<"reg"<<Ris.record_cloud<<".pcd";
        pcl::io::sav->PCDFile(ss.str(),*cloud,true);

        Ris.record_cloud++;
    }
    else
    {
    */
    std::stringstream sp;
    //sp<<"sav->.pcd";
    pcl::io::savePCDFile(sp.str(),*cloud,true);
    // }
}

void Widget::on_refresh_btn_clicked()
{
    this->refresh();
}

void Widget::on_ograbber_btn_clicked()
{
    if(oc_btn)
    {
        v->set_once(true);
        v->set_state(true);
        v->start();
        ui->ograbber_btn->setText("Close");
        oc_btn=false;
    }
    else
    {
        ui->ograbber_btn->setText("Open");
        oc_btn=true;
        v->set_state(false);
        v->exit();
    }
}

void Widget::on_getcloud_btn_clicked()
{
    if(getcloud_btn_isfirst)
    {
        if(v->cloud_mutex_.try_lock())
        {
            Ris.cloudin_1=v->cloud_;
            //view in vtk
            PointCloudColorHandlerCustom<PointT> src(Ris.cloudin_1,255,0,0);
            viz2->addPointCloud(Ris.cloudin_1,src,"src");//src view in v2
            ui->qvtkWidget_2->update();

            getcloud_btn_isfirst=false;
            v->cloud_mutex_.unlock();
        }
    }
    else
    {
        std::cerr<<"Update src"<<std::endl;
        //view in vtk2
        PointCloudColorHandlerCustom<PointT> src(Ris.cloudin_1,255,0,0);
        viz2->updatePointCloud(Ris.cloudin_1,src,"src");//src view in v2
        ui->qvtkWidget_2->update();

        if(v->cloud_mutex_.try_lock())
        {
            Ris.cloudin_2=v->cloud_;
            //view in vtk2
            std::cerr<<"Update tgt"<<std::endl;
            PointCloudColorHandlerCustom<PointT> tgt(Ris.cloudin_2,0,255,0);
            if(!viz2->updatePointCloud(Ris.cloudin_2,tgt,"tgt"))
                viz2->addPointCloud(Ris.cloudin_2,tgt,"tgt");//tgt view in v2

            ui->progressBar->setValue(0);
            Ris.start();//Thread...start!
            v->cloud_mutex_.unlock();
        }
    }
}

void Widget::refreshviz(CloudXYZRGBA::Ptr cloud_v,std::string* file)
{
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> whole(cloud_v);
    if(!viz->updatePointCloud(cloud_v,whole,*file))
        viz->addPointCloud(cloud_v,whole,*file);
    ui->qvtkWidget->update();
}

void Widget::refreshviz2(CloudXYZRGBA::Ptr cloud_v2)
{
}

void Widget::refreshviz3(CloudNormal::Ptr cloud_v3,bool issource)
{
    //judge whether the cloud is source or target
    if(issource)
    {
        PointCloudColorHandlerCustom<PointNormalT> srcin(cloud_v3,0,255,0);
        if(!viz3->updatePointCloud(cloud_v3,srcin,"source"))
            viz3->addPointCloud(cloud_v3,srcin,"source");
        ui->qvtkWidget_3->update();
    }
    else
    {
        PointCloudColorHandlerCustom<PointNormalT> tgtin(cloud_v3,255,0,0);
        if(!viz3->updatePointCloud(cloud_v3,tgtin,"target"))
            viz3->addPointCloud(cloud_v3,tgtin,"target");
        ui->qvtkWidget_3->update();
    }
}

//show the state of Grabber
void Widget::setState(bool state)
{
    if(state)
    {
        ui->grabber_state->setStyleSheet("color::green");
        ui->grabber_state->setText("Opening");
    }
    else
        ui->grabber_state->setText("Closed");
}

void Widget::on_surface_btn_clicked()
{
    Gsurface gs;
    gs.GreedyProcess(Ris.fullcloud);
    gs.show();
    gs.exec();
}

void Widget::on_pushButton_clicked()
{
    Gsurface gs;
    gs.show();
    gs.exec();
}

void Widget::on_pushButton_2_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open PCD file"), "", tr("PCD Files (*.pcd)"));
    //qDebug(fileName);
}
