#include <QApplication>
#include "MainWindow.h"
#include "Ros.h"
#include "components/crawler_panel.h"
#include <QStatusBar>
#include <QVBoxLayout>


MainWindow *MainWindow::s_self = nullptr;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent) {
    setWindowTitle("Lateral Gamma Scanner");
    resize(640, 480);
    m_view = new ImageView;
    auto crawler_panel = new CrawlerPanel();
    auto layout = new QVBoxLayout();
    layout->addWidget(crawler_panel);
    layout->addWidget(m_view);
    auto central_widget = new QWidget();
    central_widget->setLayout(layout);
    setCentralWidget(central_widget);
    auto statusbar = new QStatusBar;
    setStatusBar(statusbar);
    connect(crawler_panel->forward, SIGNAL(clicked()), crawler_panel->forward, SLOT(PressButton()));
    if (s_self) {
        LOG("Ops, only one instance of 'MainWindow' can be created!");
        QApplication::quit();
    }
    else {
        s_self = this; 
        LOG("MainWindow created...");
    }
}

MainWindow::~MainWindow() {
    LOG("MainWindow destroyed!");    
}
