#include "control_panel.h"
#include "lgs_action_cli.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "components/module_button.h"
#include <QHBoxLayout>
#include <QStatusBar>
#include <QWidget>
#include <vector>
#include <string>

MainWindow::MainWindow(QWidget *parent): QMainWindow(parent){
    setWindowTitle("Lateral Gamma Scanner");
    options_ = rclcpp::NodeOptions ();
    auto back_grip = new ModuleButton();
    auto extenders = new ModuleButton();
    auto front_grip = new ModuleButton();
    auto forward = new ModuleButton();
    auto backward = new ModuleButton();
    front_grip->SetCommands(std::string("front_grip_on"), std::string("front_grip_off"));
    back_grip->SetCommands(std::string("back_grip_on"), std::string("back_grip_off"));
    extenders->SetCommands(std::string("extender_on"), std::string("extender_off"));
    forward->SetCommands(std::string("forward"), std::string("stop"));
    backward->SetCommands(std::string("backward"), std::string("stop"));
    back_grip->AssignClient(lgs_ui::LGSActionClient::GetInstance(options_));
    extenders->AssignClient(lgs_ui::LGSActionClient::GetInstance(options_));
    front_grip->AssignClient(lgs_ui::LGSActionClient::GetInstance(options_));
    forward->AssignClient(lgs_ui::LGSActionClient::GetInstance(options_));
    backward->AssignClient(lgs_ui::LGSActionClient::GetInstance(options_));
    auto gripper_on_icon = QIcon("/home/josue/ros2_ws/src/lgs_ui/src/components/icons/gripper_on.PNG");
    auto gripper_off_icon = QIcon("/home/josue/ros2_ws/src/lgs_ui/src/components/icons/gripper_off.PNG");
    auto extender_on_icon = QIcon("/home/josue/ros2_ws/src/lgs_ui/src/components/icons/extender_on.PNG");
    auto extender_off_icon = QIcon("/home/josue/ros2_ws/src/lgs_ui/src/components/icons/extender_off.PNG");
    auto forward_icon = QIcon("/home/josue/ros2_ws/src/lgs_ui/src/components/icons/forwards.png");
    auto backward_icon = QIcon("/home/josue/ros2_ws/src/lgs_ui/src/components/icons/backwards.png");
    back_grip->SetIcons(gripper_on_icon, gripper_off_icon);
    extenders->SetIcons(extender_on_icon, extender_off_icon);
    front_grip->SetIcons(gripper_on_icon, gripper_off_icon);
    forward->SetIcons(forward_icon, forward_icon);
    backward->SetIcons(backward_icon, backward_icon);
    auto statusar = new QStatusBar;
    auto layout = new QHBoxLayout();
    auto widget = new QWidget();
    layout->addWidget(forward);
    layout->addWidget(front_grip);
    layout->addWidget(extenders);
    layout->addWidget(back_grip);
    layout->addWidget(backward);
    widget->setLayout(layout);
    connect(forward, SIGNAL(clicked()), forward, SLOT(PressButton()));
    connect(back_grip, SIGNAL(clicked()), back_grip, SLOT(PressButton()));
    connect(extenders, SIGNAL(clicked()), extenders, SLOT(PressButton()));
    connect(front_grip, SIGNAL(clicked()), front_grip, SLOT(PressButton()));
    connect(backward, SIGNAL(clicked()), backward, SLOT(PressButton()));
    setCentralWidget(widget);
    setStatusBar(statusar);
    centralWidget()->layout();
}

MainWindow::~MainWindow(){
}
 