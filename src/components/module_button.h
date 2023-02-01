#ifndef MODULEBUTTON_H
#define MODULEBUTTON_H

#include <QPushButton>
#include "../lgs_action_cli.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include <vector>

class ModuleButton : public QPushButton{
    Q_OBJECT

public:
    ModuleButton(QWidget *parent = nullptr);
    ~ModuleButton();
    int CurrentSignal();
    void SetIcons(QIcon &standby_icon, QIcon &engaged_icon);
    void AssignClient(lgs_ui::LGSActionClient * client);
    void SetCodes(int on_code = 0, int off_code = 0);
    void SetPattern(std::vector<signed short> pattern);
    void SetSingle(bool is_single_module);

public slots:
    void PressButton();

private slots:

protected:

private:
    void ReverseState();
    void CallClient();
    void PaintIcon();
    lgs_ui::LGSActionClient * client_;
    QIcon standby_icon_;
    QIcon engaged_icon_;
    bool engaged_;
    bool single_ = true;
    int on_code_;
    int off_code_;
    std::vector<signed short> pattern_;
};

#endif // MODULEBUTTON_H
