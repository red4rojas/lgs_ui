#include "Ros.h"
#include <thread>
#include <signal.h>
#include "MainWindow.h"
#include <cv_bridge/cv_bridge.h>

void kill(int /*sig*/) {
    MainWindow::instance()->close();
}

Ros *Ros::s_self = nullptr;

Ros::Ros(int argc, char *argv[], const std::string &node_name) {
    if (s_self) {
        LOG("Ops, only one instance of 'Ros' can be created!");
        MainWindow::instance()->close();
    }
    else {
        s_self = this; 
        LOG("Ros created...");
    }
    // Initilize ROS
    rclcpp::init(argc, argv);
    // Create ROS executer and node
    m_executor = rclcpp::executors::StaticSingleThreadedExecutor::make_shared();
    m_node = rclcpp::Node::make_shared(node_name);
    m_executor->add_node(m_node);
    // Add ROS publisher and subscribers
    m_front_im = m_node->create_subscription<sensor_msgs::msg::Image>("reel_image",
                                                        rclcpp::SystemDefaultsQoS(),
                                                        std::bind(&Ros::imageCallback, this, std::placeholders::_1));
    m_reel_im = m_node->create_subscription<sensor_msgs::msg::Image>("front_image",
                                                        rclcpp::SystemDefaultsQoS(),
                                                        std::bind(&Ros::imageCallback2, this, std::placeholders::_1));
    m_state_update = m_node->create_subscription<lgs_interfaces::msg::Crawlerstate>("crawler_state",
                                                        rclcpp::SystemDefaultsQoS(),
                                                        std::bind(&Ros::stateCallback, this, std::placeholders::_1));                                                        
    m_publisher = m_node->create_publisher<std_msgs::msg::String>("lgs_actuation_requests", 10);
    // int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
    // double fps = 30.0;
    // cv::Size frame_size(320, 240);
    m_writer_1 = cv::VideoWriter("/home/josue/test.mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 30, cv::Size(320,240), true);
    m_recording = false;
    signal(SIGINT, kill);
}

Ros::~Ros() {
    rclcpp::shutdown(); 
    LOG("ROS shutdown!");
}

void Ros::spin(void) {
    m_executor->spin();
}

void Ros::spinOnBackground(void) {
    std::thread thread(std::bind(&rclcpp::executors::StaticSingleThreadedExecutor::spin, m_executor));
    thread.detach();
}

void Ros::addWatcher(IWatcher * w) {
    m_watchers.push_back(w);
}

void Ros::shutdown(void) {
    m_executor->cancel();
}

void Ros::startRecording(std::string filename){
    m_writer_1 = cv::VideoWriter(filename, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 30, cv::Size(320,240), true);
    m_recording = true;
    LOG("Recording Started");
}

void Ros::stopRecording(void){
    LOG("Stopped");
    m_recording = false;
}

void Ros::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)  { 
    // LOG("Received Image: %s %dx%d", msg->encoding.c_str(), msg->width, msg->height);
    cv::Mat image_in = cv_bridge::toCvShare(msg, "bgr8")->image;
    if (m_recording) m_writer_1.write(image_in);
    if (msg->encoding != "rgb8") {
        // LOG("converting from: %s to %s", msg->encoding.c_str(), "rgb8");
        cv::cvtColor(image_in, image_in, cv::COLOR_BGR2RGB);
        if (!image_in.isContinuous() || image_in.elemSize() % 4 != 0) {
          cv::Mat aligned_image;
          image_in.copyTo(aligned_image);
          image_in = aligned_image;
        }
        QImage image(image_in.data, image_in.cols, image_in.rows, QImage::Format_RGB888);
        MainWindow::instance()->view(0)->update(image);
    } else {
        QImage image(&msg->data[0], msg->width, msg->height, QImage::Format_RGB888);
        MainWindow::instance()->view(0)->update(image);
    }
}

void Ros::imageCallback2(const sensor_msgs::msg::Image::SharedPtr msg2) const { 
    // LOG("Received Image2: %s %dx%d", msg2->encoding.c_str(), msg2->width, msg2->height);
    cv::Mat image_in2 = cv_bridge::toCvShare(msg2, "bgr8")->image;
    if (msg2->encoding != "rgb8") {
        // LOG("converting from: %s to %s", msg2->encoding.c_str(), "rgb8");
        cv::cvtColor(image_in2, image_in2, cv::COLOR_BGR2RGB);
        if (!image_in2.isContinuous() || image_in2.elemSize() % 4 != 0) {
          cv::Mat aligned_image2;
          image_in2.copyTo(aligned_image2);
          image_in2 = aligned_image2;
        }
        QImage image2(image_in2.data, image_in2.cols, image_in2.rows, QImage::Format_RGB888);
        MainWindow::instance()->view(1)->update(image2);
    } else {
        QImage image2(&msg2->data[0], msg2->width, msg2->height, QImage::Format_RGB888);
        MainWindow::instance()->view(1)->update(image2);
    }
}

void Ros::stateCallback(const lgs_interfaces::msg::Crawlerstate::SharedPtr new_state) const { 
    std::list<IWatcher*>::iterator watcher = m_watchers.begin();
    while (watcher != m_watchers.end()){
        (*watcher)->Update(new_state->state[0]);
        ++watcher;
    }
}

void Ros::publishCommand(std::string command){
  auto message = std_msgs::msg::String();
  message.data = command;
  LOG("Publishing Request: %s", command.c_str());
}