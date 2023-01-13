#include "module_button.h"

ModuleButton::ModuleButton(int on_code, int off_code, QWidget *parent):
 QPushButton(parent),
 on_code_{on_code},
 off_code_{off_code}
{
 engaged_ = false;
}

ModuleButton::~ModuleButton(){
}

void ModuleButton::SetIcons(QIcon &standby_icon, QIcon &engaged_icon){
    standby_icon_ = standby_icon;
    engaged_icon_ = engaged_icon;
    PaintIcon();
}

void ModuleButton::PaintIcon(){
    // engaged_ ? setIcon(*engaged_icon_) : setIcon(*standby_icon_);
    if (engaged_){
        setIcon(standby_icon_);
    } else {
        setIcon(engaged_icon_);
    }
    setIconSize(QSize(90, 90));
}

int ModuleButton::ButtonSignal(){
    std::cout << "buttonsignal" << std::endl;
    if (engaged_){
        return off_code_;
    } else {
        return on_code_;
    }
}

void ModuleButton::ReverseState(){
    std::cout << "reversestate" << std::endl;    
    engaged_ = !engaged_;
    ModuleButton::PaintIcon();
}