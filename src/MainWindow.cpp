#include <QApplication>
#include "MainWindow.h"
#include "Ros.h"
#include "components/BTVisualizer.h"
#include <QDir>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QStackedWidget>
#include <QDockWidget>
#include <QLabel>
#include <QDateTime>
#include <QTabWidget>

MainWindow *MainWindow::s_self = nullptr;

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
    media_path = "/home/josue/media/";
    if (s_self) {
        LOG("Ops! only one instance of 'MainWindow' can be created!");
        QApplication::quit();
    } else {
        s_self = this; 
        LOG("MainWindow created...");
    }
    setWindowTitle("Lateral Gamma Scanner");

    // // Current Inspection Tab
    m_view = new ImageView;
    m_view_2 = new ImageView;
    b_capture = new QPushButton;
    b_record = new QPushButton;
    b_stop = new QPushButton;
    auto bt_visualizer = new BTVisualizer();
    b_capture->setIcon(QIcon("/home/josue/ros2_ws/src/lgs_ui/src/components/icons/shoot.PNG"));
    b_record->setIcon(QIcon("/home/josue/ros2_ws/src/lgs_ui/src/components/icons/record.PNG"));
    b_stop->setIcon(QIcon("/home/josue/ros2_ws/src/lgs_ui/src/components/icons/stop.PNG"));    
    b_override = new ModuleButton(Ros::instance()->override_publisher(), std::string("RESUME_BT"), std::string("STOP_BT"));
    b_stop->setEnabled(false);    
    b_capture->setIconSize(QSize(200, 50));
    b_record->setIconSize(QSize(200, 50));
    b_stop->setIconSize(QSize(200, 50));
    auto recording_layout = new QVBoxLayout;
    recording_layout->addWidget(b_capture);
    recording_layout->addWidget(b_record);
    recording_layout->addWidget(b_stop);
    recording_layout->addSpacing(100);
    recording_layout->addWidget(bt_visualizer);
    recording_layout->addWidget(b_override);
    auto reel_label = new QLabel(m_view);
    auto front_label = new QLabel(m_view_2);
    reel_label->setText("Reel Video");
    front_label->setText("Front Video");
    auto recording_widget = new QWidget;
    recording_widget->setLayout(recording_layout);
    recording_widget->setMaximumWidth(400);
    auto current_inspection_layout = new QHBoxLayout();
    current_inspection_layout->addWidget(m_view);
    current_inspection_layout->addWidget(m_view_2);
    current_inspection_layout->addWidget(recording_widget);
    auto current_inspection_tab = new QWidget();
    current_inspection_tab->setLayout(current_inspection_layout);

    // // Past Inspections tab
    m_player = new MediaView;
    video_list = new QListWidget();
    video_list->setMaximumWidth(200);
    auto past_inspection_tab = new QWidget();
    auto past_inspections_layout = new QHBoxLayout();
    past_inspections_layout->addWidget(m_player);
    past_inspections_layout->addWidget(video_list);
    past_inspection_tab->setLayout(past_inspections_layout);

    //Central Widget
    auto tab_widget = new QTabWidget;
    tab_widget->insertTab(0, current_inspection_tab, "Current Inspection");
    tab_widget->insertTab(1, past_inspection_tab, "Past Inspections");
    setCentralWidget(tab_widget);
    // // Crawler Dock
    auto front_grip = new ModuleButton(Ros::instance()->actuation_publisher(), std::string("front_grip_on"), std::string("front_grip_off"));
    auto back_grip = new ModuleButton(Ros::instance()->actuation_publisher(), std::string("back_grip_on"), std::string("back_grip_off"));
    auto extenders = new ModuleButton(Ros::instance()->actuation_publisher(), std::string("extender_on"), std::string("extender_off"));
    auto forward = new ModuleButton(Ros::instance()->actuation_publisher(), std::string("forward"), std::string("stop"));
    auto backward = new ModuleButton(Ros::instance()->actuation_publisher(), std::string("backward"), std::string("stop"));
    auto pull = new ModuleButton(Ros::instance()->actuation_publisher(), std::string("pull_tether"), std::string("stop"));
    auto unwind = new ModuleButton(Ros::instance()->actuation_publisher(), std::string("unwind_tether"), std::string("stop"));
    auto crawler_dock = new QDockWidget(tr("Crawler Override Controls"),this);
    auto crawler_layout = new QHBoxLayout();
    crawler_layout->addWidget(pull);
    crawler_layout->addWidget(backward);
    crawler_layout->addWidget(back_grip);
    crawler_layout->addWidget(extenders);
    crawler_layout->addWidget(front_grip);
    crawler_layout->addWidget(forward);
    crawler_layout->addWidget(unwind);
    auto crawler_panel = new QWidget();
    crawler_panel->setLayout(crawler_layout);
    crawler_dock->setWidget(crawler_panel);
    addDockWidget(Qt::BottomDockWidgetArea, crawler_dock);
    
    // Satus Bar
    m_statusbar = new StatusBar();
    setStatusBar(m_statusbar);
    m_statusbar->showMessage("Welcome to the FIU-WRPS Lateral Gamma Scanner");

    // Connections
    connect(front_grip, SIGNAL(clicked()), this, SLOT(overrideBT()));
    connect(back_grip, SIGNAL(clicked()), this, SLOT(overrideBT()));
    connect(extenders, SIGNAL(clicked()), this, SLOT(overrideBT()));
    connect(forward, SIGNAL(clicked()), this, SLOT(overrideBT()));
    connect(backward, SIGNAL(clicked()), this, SLOT(overrideBT()));
    connect(b_record, SIGNAL(clicked()), this, SLOT(startRecording()));
    connect(b_stop, SIGNAL(clicked()), this, SLOT(stopRecording()));
    connect(video_list, SIGNAL(itemClicked(QListWidgetItem *)), this, SLOT(playVideos(QListWidgetItem*)));
    findVideos();
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
    findVideos();
}

void MainWindow::overrideBT(){
    if (b_override->GetCommand() == "STOP_BT"){
        Ros::instance()->publishCommand("stop");
        b_override->PublishCommand();
        b_override->ReverseState();
    }
}

void MainWindow::playVideos(QListWidgetItem * video){
    auto video_loc = video->text();
    m_player->update(video_loc);
    // m_player_2->update(video_loc);
}

void MainWindow::findVideos(){
    video_list->clear();
    QStringList namefilter;
    namefilter <<"*.mp4";
    QDir dir(getPath());
        for (const QString &filename : dir.entryList(namefilter,QDir::Files)){
            video_list->addItem(filename);
        }
}

QString MainWindow::fileName(){
    QString file_name = getPath();
    file_name += "/"+ QDateTime::currentDateTime().toString("yy-MM-dd_hh~mm") + ".mp4";
    return file_name;
}