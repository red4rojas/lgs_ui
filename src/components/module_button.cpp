#include "module_button.h"

ModuleButton::ModuleButton(QWidget *parent):QPushButton(parent){
    engaged_ = false;
    publisher_ = Ros::instance()->publisher();
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


void ModuleButton::CallClient(){
        auto message = std_msgs::msg::String();
        message.data = GetCommand();
        publisher_->publish(message);
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


