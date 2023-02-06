#include "module_button.h"
#include "../communicator.h"

ModuleButton::ModuleButton(QWidget *parent):
    QPushButton(parent)
{
    engaged_ = false;
}

ModuleButton::~ModuleButton(){
}

void ModuleButton::PressButton(){
    CallClient();
    ReverseState();
}

void ModuleButton::SetCommands(Command on_cmd, Command off_cmd){
    on_cmd_ = on_cmd;
    off_cmd_ = off_cmd;    
}

void ModuleButton::SetIcons(QIcon &standby_icon, QIcon &engaged_icon){
    standby_icon_ = standby_icon;
    engaged_icon_ = engaged_icon;
    PaintIcon();
}

void ModuleButton::AssignClient(lgs_ui::LGSActionClient * client){
    client_ = client;
}

void ModuleButton::CallClient(){
        client_->pass_command(GetCommand());
}

void ModuleButton::PaintIcon(){
    if (engaged_){
        setIcon(standby_icon_);
    } else {
        setIcon(engaged_icon_);
    }
    setIconSize(QSize(120, 120));
}

std::string ModuleButton::GetCommand(){
    if (engaged_){
        return off_cmd_;
    } else {
        return on_cmd_;
    }
}

void ModuleButton::ReverseState(){
    engaged_ = !engaged_;
    ModuleButton::PaintIcon();
}


