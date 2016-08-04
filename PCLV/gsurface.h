#ifndef GSURFACE_H
#define GSURFACE_H

#include <QDialog>
#include "openni_grabber1.h"
#include "registration1.h"
#include <pcl/surface/gp3.h>

namespace Ui {
class Gsurface;
}

class Gsurface : public QDialog
{
    Q_OBJECT
    
public:
    explicit Gsurface(QWidget *parent = 0);
    ~Gsurface();
    void GreedyProcess(CloudXYZRGBA::Ptr cloud_in);
    
private:
    Ui::Gsurface *ui;
};

#endif // GSURFACE_H
