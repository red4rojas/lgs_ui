#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "components/ImageView.h"
#include <QListWidget>
#include <QPushButton>
#include <QStatusBar>


class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    static MainWindow *instance() { return s_self; };
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    ImageView *view(int aux = 0) {return aux ? m_view_2 : m_view;}
    QString getDir();
private slots:
    void startRecording();
    void stopRecording();
    QString fileName();
private:
    // Data
    ImageView *m_view;
    ImageView *m_view_2;
    static MainWindow *s_self;
    QListWidget *video_list;
    QStatusBar  *m_statusbar;
    QPushButton * b_capture;
    QPushButton * b_record;
    QPushButton * b_stop;
};

#endif // MAINWINDOW_H
