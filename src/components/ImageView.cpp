#include "ImageView.h"
#include <QPainter>
#include <iostream>
#include <QDebug>
#include <QResizeEvent>
#include <QFileInfo>
#include <QUrl> 
#include <QPaintEvent>

ImageView::ImageView(QWidget *parent) :
    QWidget(parent) {
}

QImage scale(const QImage &image, QSize size, bool keepAspectioRatio = true) {
    QImage scaled;
    if (image.isNull() or size.isNull() )
        return scaled;
    if (keepAspectioRatio) {
        if (size.height() < size.width())
            scaled = image.scaledToHeight(size.height());
        else
            scaled = image.scaledToWidth(size.width());
    }
    else
        scaled = image.scaled(size);
    return scaled;
}

QRect center(QSize frame, QSize object) {
    int w = frame.width();
    int h = frame.height();
    int iw = object.width();
    int ih = object.height();
    if (w > h) {
        iw = (int)((float)h*iw/ih + 0.5f);
        ih = h;
    }
    else {
        ih = (int)((float)w*ih/iw + 0.5f);
        iw = w;
    }
    return QRect((w-iw)/2, (h-ih)/2, iw, ih);
}

void ImageView::update(const QImage &image) {
    m_image = image;
    QWidget::update();
}

void ImageView::paintEvent(QPaintEvent *) {
    if (m_image.isNull())
        return;
    QPainter painter(this);
    painter.drawImage(center(size(), m_image.size()), m_image);
}
