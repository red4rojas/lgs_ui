#include "module_button.h"
#include "../communicator.h"

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

void ModuleButton::AssignClient(lgs_ui::LGSActionClient * client){
    client_ = client;
}

void ModuleButton::PressButton(){
    CallClient();
    ReverseState();
}
void ModuleButton::CallClient(){
    client_->send_goal((short) CurrentSignal());
}

void ModuleButton::PaintIcon(){
    if (engaged_){
        setIcon(standby_icon_);
    } else {
        setIcon(engaged_icon_);
    }
    setIconSize(QSize(120, 120));
    }

int ModuleButton::CurrentSignal(){
    if (engaged_){
        return off_code_;
    } else {
        return on_code_;
    }
}

void ModuleButton::ReverseState(){
    engaged_ = !engaged_;
    ModuleButton::PaintIcon();
}