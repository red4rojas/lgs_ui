#ifndef MEDAIVIEW_H
#define MEDAIVIEW_H

#include <QObject>
#include <QVideoWidget>
#include <QMediaPlayer>

class MediaView : public QVideoWidget {
    Q_OBJECT
public:
    MediaView(QWidget *parent = nullptr);
    void update(const QString &filename);
protected:
private:
    QMediaPlayer *m_player;
};

#endif // MEDAIVIEW_H
