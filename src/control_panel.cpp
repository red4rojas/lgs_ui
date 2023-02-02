#include "control_panel.h"
#include "lgs_action_cli.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "components/module_button.h"
#include <QHBoxLayout>
#include <QStatusBar>
#include <QWidget>
#include <vector>

MainWindow::MainWindow(QWidget *parent): QMainWindow(parent){
    setWindowTitle("Lateral Gamma Scanners");
    options_ = rclcpp::NodeOptions ();
    auto back_grip_b = new ModuleButton();
    auto extenders_b = new ModuleButton();
    auto front_grip_b = new ModuleButton();
    auto forward_b = new ModuleButton();
    auto backward_b = new ModuleButton();
    forward_b->SetSingle(false);
    backward_b->SetSingle(false);
    back_grip_b->SetCodes(1,2);
    extenders_b->SetCodes(3,4);
    front_grip_b->SetCodes(5,6);
    std::vector<signed short> forward_pattern{1,3,5,2,4,6};
    std::vector<signed short> backward_pattern{5,3,1,6,4,2};
    forward_b->SetPattern(forward_pattern);
    backward_b->SetPattern(backward_pattern);
    back_grip_b->AssignClient(lgs_ui::LGSActionClient::GetInstance(options_));
    extenders_b->AssignClient(lgs_ui::LGSActionClient::GetInstance(options_));
    front_grip_b->AssignClient(lgs_ui::LGSActionClient::GetInstance(options_));
    forward_b->AssignClient(lgs_ui::LGSActionClient::GetInstance(options_));
    backward_b->AssignClient(lgs_ui::LGSActionClient::GetInstance(options_));
    auto gripper_on_icon = QIcon("/home/josue/ros2_ws/src/lgs_ui/src/components/icons/gripper_on.PNG");
    auto gripper_off_icon = QIcon("/home/josue/ros2_ws/src/lgs_ui/src/components/icons/gripper_off.PNG");
    auto extender_on_icon = QIcon("/home/josue/ros2_ws/src/lgs_ui/src/components/icons/extender_on.PNG");
    auto extender_off_icon = QIcon("/home/josue/ros2_ws/src/lgs_ui/src/components/icons/extender_off.PNG");
    auto forward_icon = QIcon("/home/josue/ros2_ws/src/lgs_ui/src/components/icons/forwards.png");
    auto backward_icon = QIcon("/home/josue/ros2_ws/src/lgs_ui/src/components/icons/backwards.png");
    back_grip_b->SetIcons(gripper_on_icon, gripper_off_icon);
    extenders_b->SetIcons(extender_on_icon, extender_off_icon);
    front_grip_b->SetIcons(gripper_on_icon, gripper_off_icon);
    forward_b->SetIcons(forward_icon, forward_icon);
    backward_b->SetIcons(backward_icon, backward_icon);
    auto status_bar = new QStatusBar;
    auto layout = new QHBoxLayout();
    auto widget = new QWidget();
    layout->addWidget(forward_b);
    layout->addWidget(front_grip_b);
    layout->addWidget(extenders_b);
    layout->addWidget(back_grip_b);
    layout->addWidget(backward_b);
    widget->setLayout(layout);
    connect(forward_b, SIGNAL(clicked()), forward_b, SLOT(PressButton()));
    connect(back_grip_b, SIGNAL(clicked()), back_grip_b, SLOT(PressButton()));
    connect(extenders_b, SIGNAL(clicked()), extenders_b, SLOT(PressButton()));
    connect(front_grip_b, SIGNAL(clicked()), front_grip_b, SLOT(PressButton()));
    connect(backward_b, SIGNAL(clicked()), backward_b, SLOT(PressButton()));
    setCentralWidget(widget);
    setStatusBar(status_bar);
    centralWidget()->layout();
}

MainWindow::~MainWindow(){
}
 