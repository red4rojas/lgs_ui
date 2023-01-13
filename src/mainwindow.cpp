#include "mainwindow.h"
#include "lgs_action_cli.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "components/module_button.h"
#include <QHBoxLayout>
#include <QWidget>


MainWindow::MainWindow(QWidget *parent): QMainWindow(parent){
    setWindowTitle("Lateral Gamma Scanners");
    options_ = rclcpp::NodeOptions ();
    client_ = lgs_ui::LGSActionClient::GetInstance(options_);
    testbutton_ = new ModuleButton(1,2);
    onIcon = QIcon("/home/josue/ros2_ws/src/lgs_ui/src/components/icons/gripper_on.PNG");
    offIcon = QIcon("/home/josue/ros2_ws/src/lgs_ui/src/components/icons/gripper_off.PNG");

    testbutton_->SetIcons(onIcon, offIcon);
    status_bar_ = new QStatusBar;

    layout_ = new QHBoxLayout();
    layout_->addWidget(testbutton_);
    widget_ = new QWidget();
    widget_->setLayout(layout_);
    connect(testbutton_, SIGNAL(clicked()), this, SLOT(SendIt()));
    setCentralWidget(widget_);
    setStatusBar(status_bar_);
}

MainWindow::~MainWindow(){
}

void MainWindow::SendIt(){
    std::cout << "sendIt" << std::endl;
    client_->send_goal((short) testbutton_->ButtonSignal());
    testbutton_->ReverseState();
}