#include "crawler_panel.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "module_button.h"
#include <QHBoxLayout>
#include <QWidget>
#include <vector>
#include <string>
#include <iostream>

CrawlerPanel::CrawlerPanel(QWidget *parent): QWidget(parent){
    std::cout << "Crawler Panel Created" << std::endl;
    setWindowTitle("Lateral Gamma Scanner");
    back_grip = new ModuleButton();
    extenders = new ModuleButton();
    front_grip = new ModuleButton();
    forward = new ModuleButton();
    backward = new ModuleButton();
    front_grip->SetCommands(std::string("front_grip_on"), std::string("front_grip_off"));
    back_grip->SetCommands(std::string("back_grip_on"), std::string("back_grip_off"));
    extenders->SetCommands(std::string("extender_on"), std::string("extender_off"));
    forward->SetCommands(std::string("forward"), std::string("stop"));
    backward->SetCommands(std::string("backward"), std::string("stop"));
    auto gripper_on_icon = QIcon("/home/josue/ros2_ws/src/lgs_ui/src/components/icons/gripper_on.PNG");
    auto gripper_off_icon = QIcon("/home/josue/ros2_ws/src/lgs_ui/src/components/icons/gripper_off.PNG");
    auto extender_on_icon = QIcon("/home/josue/ros2_ws/src/lgs_ui/src/components/icons/extender_on.PNG");
    auto extender_off_icon = QIcon("/home/josue/ros2_ws/src/lgs_ui/src/components/icons/extender_off.PNG");
    auto forward_icon = QIcon("/home/josue/ros2_ws/src/lgs_ui/src/components/icons/forwards.png");
    auto backward_icon = QIcon("/home/josue/ros2_ws/src/lgs_ui/src/components/icons/backwards.png");
    auto stop_icon = QIcon("/home/josue/ros2_ws/src/lgs_ui/src/components/icons/stop.png");
    back_grip->SetIcons(gripper_on_icon, gripper_off_icon);
    extenders->SetIcons(extender_on_icon, extender_off_icon);
    front_grip->SetIcons(gripper_on_icon, gripper_off_icon);
    forward->SetIcons(stop_icon, forward_icon);
    backward->SetIcons(stop_icon, backward_icon);
    auto layout = new QHBoxLayout();
    layout->addWidget(forward);
    layout->addWidget(front_grip);
    layout->addWidget(extenders);
    layout->addWidget(back_grip);
    layout->addWidget(backward);
    setLayout(layout);
    connect(forward, SIGNAL(clicked()), forward, SLOT(PressButton()));
    connect(back_grip, SIGNAL(clicked()), back_grip, SLOT(PressButton()));
    connect(extenders, SIGNAL(clicked()), extenders, SLOT(PressButton()));
    connect(front_grip, SIGNAL(clicked()), front_grip, SLOT(PressButton()));
    connect(backward, SIGNAL(clicked()), backward, SLOT(PressButton()));
}

CrawlerPanel::~CrawlerPanel(){
}
 