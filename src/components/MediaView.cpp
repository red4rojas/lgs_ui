#include "MediaView.h"
#include <QPainter>
#include <iostream>
#include <QDebug>
#include <QResizeEvent>
#include <QFileInfo>
#include <QUrl> 
#include <QPaintEvent>

MediaView::MediaView(QWidget *parent) :
    QVideoWidget(parent) {
}

void MediaView::update(const QString &filename) {
    qDebug() << "Button clicked";
    QImage image;
    QString fullpath = "/home/josue/media/";
    fullpath += filename;
    if (!QFileInfo::exists(fullpath)) {
        qDebug() << "Ops, cannot play file," << fullpath << "doesn't exit!";
        return;
    }
    m_player = new QMediaPlayer();
    auto dir = QUrl(fullpath);
    m_player->setSource(fullpath);
    m_player->setVideoOutput( this );
    m_player->play();
}

