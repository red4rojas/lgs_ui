#include <QApplication>
#include "MainWindow.h"
#include "Ros.h"
#include "components/module_button.h"
#include <QDir>
#include <QVBoxLayout>
#include <QStackedWidget>
#include <QToolBox>
#include <QDockWidget>
#include <QHBoxLayout>
#include <QDateTime>

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
    auto crawler_dock = new QDockWidget(tr("Crawler Controls"),this);
    auto crawler_panel = new QWidget();
    crawler_panel->setLayout(crawler_layout);
    crawler_dock->setWidget(crawler_panel);
    auto media_dock = new QDockWidget(tr ("Videos"), this);
    auto media_toolbox = new QToolBox();
    video_list = new QListWidget();
    b_capture = new QPushButton ;
    b_record = new QPushButton ;
    b_stop = new QPushButton;
    b_capture->setIcon(QIcon("/home/josue/ros2_ws/src/lgs_ui/src/components/icons/shoot.PNG"));
    b_record->setIcon(QIcon("/home/josue/ros2_ws/src/lgs_ui/src/components/icons/record.PNG"));
    b_stop->setIcon(QIcon("/home/josue/ros2_ws/src/lgs_ui/src/components/icons/stop.PNG"));    
    b_stop->setEnabled(false);    
    b_capture->setIconSize(QSize(200, 50));
    b_record->setIconSize(QSize(200, 50));
    b_stop->setIconSize(QSize(200, 50));
    auto recording_layout = new QVBoxLayout;
    auto recording_widget = new QWidget;
    recording_widget->setLayout(recording_layout);
    recording_layout->addWidget(b_capture);
    recording_layout->addWidget(b_stop);
    recording_layout->addWidget(b_record);
    media_toolbox->addItem(recording_widget, "Record New");
    media_toolbox->addItem(video_list, "Past recordings");
    media_dock->setWidget(media_toolbox);
    auto videos_layout = new QHBoxLayout();
    auto main_layout = new QVBoxLayout();
    m_statusbar = new QStatusBar();
    auto central_widget = new QWidget();
    auto videos_widget = new QWidget();
    videos_layout->addWidget(m_view);
    videos_layout->addWidget(m_view_2);
    videos_widget->setLayout(videos_layout);
    main_layout->addWidget(videos_widget);
    central_widget->setLayout(main_layout);
    addDockWidget(Qt::RightDockWidgetArea,media_dock);
    addDockWidget(Qt::BottomDockWidgetArea,crawler_dock);
    setCentralWidget(central_widget);
    setStatusBar(m_statusbar);
    connect(b_record, SIGNAL(clicked()), this, SLOT(startRecording()));
    connect(b_stop, SIGNAL(clicked()), this, SLOT(stopRecording()));
}

MainWindow::~MainWindow() {
    LOG("MainWindow destroyed!");    
}

QString MainWindow::getDir(){
    QString app_dir = QCoreApplication::applicationDirPath();
    QString source_path = __FILE__;
    QString source_relpath = QDir::cleanPath(source_path.remove(app_dir));
    QString source_dir = QFileInfo(source_relpath).dir().path();   
    return source_dir;
}

void MainWindow::startRecording(){
    Ros::instance()->startRecording(fileName().toStdString());
    b_record->setEnabled(false);
    b_stop->setEnabled(true);

}

void MainWindow::stopRecording(){
    Ros::instance()->stopRecording();
    b_stop->setEnabled(false);
    b_record->setEnabled(true);
}

QString MainWindow::fileName(){
    QString file_name = "/home/josue/media";
    file_name += "/"+ QDateTime::currentDateTime().toString("yy-MM-dd_hh~mm") + ".mp4";
    return file_name;
}