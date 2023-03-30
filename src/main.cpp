#include "MainWindow.h"
#include <QApplication>
#include "rclcpp/rclcpp.hpp"
#include "Ros.h"

int main(int argc, char *argv[])
{
    Ros ros(argc, argv, "user_interface");
    QApplication a(argc, argv);
    MainWindow w;
    w.resize(1080,720);
    w.show();
    ros.spinOnBackground();
    return a.exec();
}
