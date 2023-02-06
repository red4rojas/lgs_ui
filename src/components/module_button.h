#ifndef MODULEBUTTON_H
#define MODULEBUTTON_H

#include <QPushButton>
#include "../lgs_action_cli.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include <vector>

class ModuleButton : public QPushButton{
    Q_OBJECT

public:
    using Command = std::string;
    ModuleButton(QWidget *parent = nullptr);
    ~ModuleButton();
    void SetCommands(std::string on, std::string off);
    void AssignClient(lgs_ui::LGSActionClient * client);
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
    lgs_ui::LGSActionClient * client_;
    QIcon standby_icon_;
    QIcon engaged_icon_;
    std::string on_cmd_;
    std::string off_cmd_;
    bool engaged_;
};

#endif // MODULEBUTTON_H
