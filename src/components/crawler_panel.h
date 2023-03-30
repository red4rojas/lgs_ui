#ifndef CRAWLERPANEL_H
#define CRAWLERPANEL_H

#include <QWidget>
#include <QPushButton>

#include "module_button.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class CrawlerPanel: public QWidget{
    Q_OBJECT

public:
    CrawlerPanel(QWidget *parent = nullptr);
    ~CrawlerPanel();
    ModuleButton * back_grip;
    ModuleButton * extenders;
    ModuleButton * front_grip;
    ModuleButton * forward;
    ModuleButton * backward;
public slots:

private slots:

protected:

private:
    rclcpp::NodeOptions options_;
};

#endif // CRAWLERPANEL_H
