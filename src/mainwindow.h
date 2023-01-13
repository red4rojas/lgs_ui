#ifndef MAINWINDOW_H
#define MAINWINDOW_H

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
    void SendIt();

private slots:

protected:

private:
    rclcpp::NodeOptions options_;
    lgs_ui::LGSActionClient *client_;
    ModuleButton *testbutton_;
    QHBoxLayout *layout_;
    QWidget *widget_;
    QStatusBar *status_bar_;
    QIcon onIcon;
    QIcon offIcon;
};

#endif // MAINWINDOW_H
