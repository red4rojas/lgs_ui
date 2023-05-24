#include "BTVisualizer.h"
#include <QCoreApplication>
#include <QDir>


// BTVisualizer::BTVisualizer(QWidget *parent){
BTVisualizer::BTVisualizer(QWidget *parent):
    QLabel(parent){
    Ros::instance()->addWatcher(this);
    pix = QPixmap("/home/josue/ros2_ws/src/lgs_ui/src/components/icons/forward.PNG");
    setPixmap(pix.scaled(this->width()*0.4,80,Qt::KeepAspectRatioByExpanding));
}

void BTVisualizer::Update(const std::string signal){
    QString app_dir = QCoreApplication::applicationDirPath();
    QString source_path = __FILE__;
    QString source_relpath = QDir::cleanPath(source_path.remove(app_dir));
    QString source_dir = QFileInfo(source_relpath).dir().path();
    pix.load(source_dir.append(QString::fromStdString("/icons/" + signal + ".PNG")));
    setPixmap(pix.scaled(this->width(),100,Qt::KeepAspectRatioByExpanding));
    return;
}