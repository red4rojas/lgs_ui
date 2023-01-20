#include "control_panel.h"
#include <QApplication>
#include "rclcpp/rclcpp.hpp"
// #include "lgs_action_cli.cpp"


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    rclcpp::init(argc, argv) ;
    MainWindow w;
    w.resize(1080,720);
    w.show();
    return a.exec();
}
