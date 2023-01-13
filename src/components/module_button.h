#ifndef MODULEBUTTON_H
#define MODULEBUTTON_H

#include <QPushButton>
#include "rclcpp/rclcpp.hpp"
#include "../lgs_action_cli.h"
#include "rclcpp_action/rclcpp_action.hpp"

class ModuleButton : public QPushButton{
    Q_OBJECT

public:
    ModuleButton(int on_code = 0, int off_code = 0, QWidget *parent = nullptr);
    ~ModuleButton();
    void SetIcons(QIcon &standby_icon, QIcon &engaged_icon);
    int CurrentSignal();
    void ReverseState();
    void AssignClient(lgs_ui::LGSActionClient * client);
    void CallClient();

public slots:
    void PressButton();

private slots:

protected:

private:
    lgs_ui::LGSActionClient * client_;
    void PaintIcon();
    QIcon standby_icon_;
    QIcon engaged_icon_;
    bool engaged_;
    int on_code_;
    int off_code_;
};

#endif // MODULEBUTTON_H
