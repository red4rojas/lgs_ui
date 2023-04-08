#include <QApplication>
#include "MainWindow.h"
#include "Ros.h"
#include "components/module_button.h"
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

    auto front_grip = new ModuleButton(std::string("front_grip_on"), std::string("front_grip_off"));
    auto back_grip = new ModuleButton(std::string("back_grip_on"), std::string("back_grip_off"));
    auto extenders = new ModuleButton(std::string("extender_on"), std::string("extender_off"));
    auto forward = new ModuleButton(std::string("forward"), std::string("stop"));
    auto backward = new ModuleButton(std::string("backward"), std::string("stop"));
    auto crawler_layout = new QHBoxLayout();
    crawler_layout->addWidget(forward);
    crawler_layout->addWidget(front_grip);
    crawler_layout->addWidget(extenders);
    crawler_layout->addWidget(back_grip);
    crawler_layout->addWidget(backward);
    auto crawler_panel = new QWidget();
    crawler_panel->setLayout(crawler_layout);
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