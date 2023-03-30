#ifndef MODULEBUTTON_H
#define MODULEBUTTON_H

#include <QPushButton>
#include "../Ros.h"
#include <vector>
#include <memory>

class ModuleButton : public QPushButton{
    Q_OBJECT

public:
    using Command = std::string;
    ModuleButton(QWidget *parent = nullptr);
    ~ModuleButton();
    void SetCommands(std::string on, std::string off);
    // void AssignNode(Ros * node);
    void SetIcons(QIcon &standby_icon, QIcon &engaged_icon);

public slots:
    void PressButton();

private slots:

protected:

private:
    void ReverseState();
    void CallClient();
    void PaintIcon();
    std::string GetCommand();
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    QIcon standby_icon_;
    QIcon engaged_icon_;
    std::string on_cmd_;
    std::string off_cmd_;
    bool engaged_;
};

#endif // MODULEBUTTON_H
