#include <QApplication>
#include "MainWindow.h"
#include "Ros.h"
#include "components/crawler_panel.h"
#include <QStatusBar>
#include <QVBoxLayout>
#include <QHBoxLayout>

MainWindow *MainWindow::s_self = nullptr;

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
    if (s_self) {
        LOG("oops! only one instance of 'MainWindow' can be created!");
        QApplication::quit();
    } else {
        s_self = this; 
        LOG("MainWindow created...");
    }
    setWindowTitle("Lateral Gamma Scanner");
    m_view = new ImageView;
    m_view_2 = new ImageView;
    auto crawler_panel = new CrawlerPanel();
    auto videos_layout = new QHBoxLayout;
    auto main_layout = new QVBoxLayout();
    auto statusbar = new QStatusBar;
    auto central_widget = new QWidget();
    auto videos_widget = new QWidget();
    videos_layout->addWidget(m_view);
    videos_layout->addWidget(m_view_2);
    videos_widget->setLayout(videos_layout);
    main_layout->addWidget(crawler_panel);
    main_layout->addWidget(videos_widget);
    central_widget->setLayout(main_layout);
    setCentralWidget(central_widget);
    setStatusBar(statusbar);
}

MainWindow::~MainWindow() {
    LOG("MainWindow destroyed!");    
}