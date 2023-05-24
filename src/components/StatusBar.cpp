#include "StatusBar.h"

StatusBar::StatusBar(QWidget *parent):
    QStatusBar(parent){
    Ros::instance()->addWatcher(this);
}

// StatusBar::~StatusBar(){
// }

void StatusBar::Update(const std::string signal){
    auto message = QString::fromStdString(signal);
    showMessage(message);
    return;
}

