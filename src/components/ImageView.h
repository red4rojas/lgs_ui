#ifndef IMAGEVIEW_H
#define IMAGEVIEW_H

#include <QObject>
#include <QWidget>
#include <QImage>

class ImageView : public QWidget {
    Q_OBJECT
public:
    ImageView(QWidget *parent = nullptr);
    void update(const QString &filename);
    void update(const QImage &image);
protected:
    void paintEvent(QPaintEvent *) override;
private:
    QImage m_image;
};

#endif // IMAGEVIEW_H
