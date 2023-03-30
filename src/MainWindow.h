#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "components/ImageView.h"
#include "components/crawler_panel.h"

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    static MainWindow *instance() { return s_self; };
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    ImageView *view() { return m_view; }

private:
    // Data
    ImageView *m_view;
    static MainWindow *s_self;
};

#endif // MAINWINDOW_H
