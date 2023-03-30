#include <QApplication>
#include "MainWindow.h"
#include "Ros.h"
#include "components/crawler_panel.h"
#include <QStatusBar>
#include <QVBoxLayout>

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
    auto crawler_panel = new CrawlerPanel();
    auto layout = new QVBoxLayout();
    auto statusbar = new QStatusBar;
    auto central_widget = new QWidget();
    layout->addWidget(crawler_panel);
    layout->addWidget(m_view);
    central_widget->setLayout(layout);
    setCentralWidget(central_widget);
    setStatusBar(statusbar);
}

MainWindow::~MainWindow() {
    LOG("MainWindow destroyed!");    
}