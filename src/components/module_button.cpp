#include "module_button.h"
#include <libgen.h>
#include <QCoreApplication>
#include <QDir>

// Icon PNG files should be named like the command string of their button
// my_command.PNG

ModuleButton::ModuleButton(std::string on_cmd, std::string off_cmd, QWidget *parent):
    QPushButton(parent),
    on_cmd_(on_cmd),
    off_cmd_(off_cmd){
    engaged_ = false;
    publisher_ = Ros::instance()->publisher();
    connect(this, SIGNAL(clicked()), this, SLOT(PressButton()));
    Ros::instance()->addWatcher(this);
    SetIcons();
}

ModuleButton::~ModuleButton(){
}

void ModuleButton::PressButton(){
    PublishCommand();
    ReverseState();
}
void ModuleButton::Update(const int& signal){
    // std::cout << signal << std::endl;
    return;
}

void ModuleButton::SetIcons(){
    QString app_dir = QCoreApplication::applicationDirPath();
    QString source_path = __FILE__;
    QString source_relpath = QDir::cleanPath(source_path.remove(app_dir));
    QString source_dir = QFileInfo(source_relpath).dir().path();
    QString standby_icon_dir = source_dir;
    QString engaged_icon_dir = source_dir;
    standby_icon_ = QIcon(standby_icon_dir.append(QString::fromStdString("/icons/" + off_cmd_ + ".PNG")));
    engaged_icon_ = QIcon(engaged_icon_dir.append(QString::fromStdString("/icons/" + on_cmd_ + ".PNG"))); 
    PaintIcon();
}

void ModuleButton::PublishCommand(){
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
