#include "openni_grabber1.h"

openni_grabber1::openni_grabber1()
{
}

void openni_grabber1::run()
{
    pcl::OpenNIGrabber *grabber_=new pcl::OpenNIGrabber();
    std::cerr<<"    run grabber therad..."<<std::endl;

    boost::function<void(const CloudXYZRGBA::ConstPtr &cloud)> f(boost::bind(&openni_grabber1::cloudCallback,this,_1));

    boost::signals2::connection c1 = grabber_->registerCallback(f);

    std::cerr<<"    grabber trying start..."<<std::endl;
    grabber_->start();
    std::cerr<<"    grabber start ok!"<<std::endl;
    while(check_state)
    {
        //std::cerr<<"            grabber stay opening"<<std::endl;
        boost::this_thread::sleep(boost::posix_time::seconds(2));
    }
    std::cerr<<"stop grabber "<<std::endl;
    grabber_->stop();
    emit grabber_state(0);
}

void openni_grabber1::cloudCallback(const CloudXYZRGBA::ConstPtr &cloud)
{
    if(cloud_mutex_.try_lock())
    {

        std::cerr<<"        callback locked"<<std::endl;

        cloud_=cloud;
        cloud_mutex_.unlock();
        //std::cerr<<"        callback unlocked"<<std::endl;

        if(check_once)
        {
            //std::cerr<<"Get Cloud!"<<std::endl;
            emit grabber_state(1);
            check_once=false;
        }
    }
    boost::this_thread::sleep(boost::posix_time::microseconds(100));
}

//when start thread set it true to check in callback function once
void openni_grabber1::set_once(bool state)
{
    check_once=state;
}

//check the Grabber state
void openni_grabber1::set_state(bool state)
{
    check_state=state;
}
