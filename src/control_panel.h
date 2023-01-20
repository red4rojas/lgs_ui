#ifndef CONTROLPANEL_H
#define CONTROLPANEL_H

#include <QMainWindow>
#include <QPushButton>
#include <QHBoxLayout>
#include <QWidget>
#include <QStatusBar>

#include "lgs_action_cli.h"
#include "components/module_button.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class MainWindow : public QMainWindow{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

public slots:

private slots:

protected:

private:
    rclcpp::NodeOptions options_;
    ModuleButton *front_grip_button_;
    ModuleButton *extenders_button_;
    ModuleButton *back_grip_button_;
    QHBoxLayout *layout_;
    QWidget *widget_;
    QStatusBar *status_bar_;
    QIcon gripper_on_icon_;
    QIcon gripper_off_icon_;
    QIcon extender_on_icon_;
    QIcon extender_off_icon_;
};

#endif // CONTROLPANEL_H
