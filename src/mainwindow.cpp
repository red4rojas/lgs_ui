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
    back_grip_button_ = new ModuleButton(1,2);
    extenders_button_ = new ModuleButton(3,4);
    front_grip_button_ = new ModuleButton(5,6);
    back_grip_button_->AssignClient(lgs_ui::LGSActionClient::GetInstance(options_));
    extenders_button_->AssignClient(lgs_ui::LGSActionClient::GetInstance(options_));
    front_grip_button_->AssignClient(lgs_ui::LGSActionClient::GetInstance(options_));
    gripper_on_icon_ = QIcon("../../../../src/lgs_ui/src/components/icons/gripper_on.PNG");
    gripper_off_icon_ = QIcon("../../../../src/lgs_ui/src/components/icons/gripper_off.PNG");
    extender_on_icon_ = QIcon("../../../../src/lgs_ui/src/components/icons/extender_on.PNG");
    extender_off_icon_ = QIcon("../../../../src/lgs_ui/src/components/icons/extender_off.PNG");
    back_grip_button_->SetIcons(gripper_on_icon_, gripper_off_icon_);
    extenders_button_->SetIcons(extender_on_icon_, extender_off_icon_);
    front_grip_button_->SetIcons(gripper_on_icon_, gripper_off_icon_);

    status_bar_ = new QStatusBar;
    layout_ = new QHBoxLayout();
    layout_->addWidget(front_grip_button_);
    layout_->addWidget(extenders_button_);
    layout_->addWidget(back_grip_button_);
    widget_ = new QWidget();
    widget_->setLayout(layout_);
    connect(back_grip_button_, SIGNAL(clicked()), back_grip_button_, SLOT(PressButton()));
    connect(extenders_button_, SIGNAL(clicked()), extenders_button_, SLOT(PressButton()));
    connect(front_grip_button_, SIGNAL(clicked()), front_grip_button_, SLOT(PressButton()));
    setCentralWidget(widget_);
    setStatusBar(status_bar_);
}

MainWindow::~MainWindow(){
}
