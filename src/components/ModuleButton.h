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
    ModuleButton(std::string on, std::string off, QWidget *parent = nullptr);
    ~ModuleButton();
    void SetIcons();

public slots:
    void PressButton();
private slots:
protected:
private:
    void Update(const std::string signal) override;
    void ReverseState();
    void PublishCommand();
    void PaintIcon();
    std::string GetCommand();
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    QIcon standby_icon_;
    QIcon engaged_icon_;
    std::string on_cmd_;
    std::string off_cmd_;
    bool engaged_;
};

#endif // MODULEBUTTON_H
