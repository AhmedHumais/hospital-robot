#include "mainwindow.h"

#include <QApplication>
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    if( !ros::isInitialized() )
    {
        ros::init( argc, argv, "cov19rob" );
    }
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}
