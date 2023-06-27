#ifndef MODULEBUTTON_H
#define MODULEBUTTON_H

#include <QPushButton>
#include "../Ros.h"
#include "IWatcher.h"
#include <vector>
#include <memory>
#include <iostream>

class ModuleButton : public QPushButton, public IWatcher{
    Q_OBJECT

public:
    ModuleButton(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher, std::string on, std::string off, QWidget *parent = nullptr);
    ~ModuleButton();
    void SetIcons();
    std::string GetCommand();

public slots:
    void PressButton();
    void ReverseState();
    void PublishCommand();
private slots:
protected:
private:
    void Update(const std::string signal) override;
    void PaintIcon();
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    QIcon standby_icon_;
    QIcon engaged_icon_;
    std::string on_cmd_;
    std::string off_cmd_;
    bool engaged_;
};

#endif // MODULEBUTTON_H
