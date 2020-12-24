/**
 * @file main.cpp
 * @author Muhammad Ahmed Humais
 *         muhammad.humais@ku.ac.ae
 *         Khalifa University
 * @brief source file for main function to initiate ros and MainWindow
 * @date 2020/09/16
 * @version v1.0
 * @package hospital_robot
 */

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
