#ifndef MODULEBUTTON_H
#define MODULEBUTTON_H

#include <QPushButton>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class ModuleButton : public QPushButton{
    Q_OBJECT

public:
    ModuleButton(int on_code = 0, int off_code = 0, QWidget *parent = nullptr);
    ~ModuleButton();
    void SetIcons(QIcon &standby_icon, QIcon &engaged_icon);
    int ButtonSignal();
    void ReverseState();

public slots:

private slots:

protected:

private:
    void PaintIcon();
    QIcon standby_icon_;
    QIcon engaged_icon_;
    bool engaged_;
    int on_code_;
    int off_code_;
};

#endif // MODULEBUTTON_H
