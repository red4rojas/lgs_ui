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
    // Initilize ROS
    rclcpp::init(argc, argv);
    // Create ROS executer and node
    m_executor = rclcpp::executors::StaticSingleThreadedExecutor::make_shared();
    m_node = rclcpp::Node::make_shared(node_name);
    m_executor->add_node(m_node);
    // Add ROS publisher and subscribers
    m_image = m_node->create_subscription<sensor_msgs::msg::Image>("image",
                                                        rclcpp::SystemDefaultsQoS(),
                                                        std::bind(&Ros::imageCallback, this, std::placeholders::_1));
    Ros::m_publisher = m_node->create_publisher<std_msgs::msg::String>("lgs_actuation_requests", 10);
    if (s_self) {
        LOG("Ops, only one instance of 'Ros' can be created!");
        MainWindow::instance()->close();
    }
    else {
        s_self = this; 
        LOG("Ros created...");
    }
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

void Ros::shutdown(void) {
    m_executor->cancel();
}

void Ros::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) const { 
    LOG("Received Image: %s %dx%d", msg->encoding.c_str(), msg->width, msg->height);
    if (msg->encoding != "rgb8") {
        LOG("converting from: %s to %s", msg->encoding.c_str(), "rgb8");
        cv::Mat image_in = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::cvtColor(image_in, image_in, cv::COLOR_BGR2RGB);
        if (!image_in.isContinuous() || image_in.elemSize() % 4 != 0) {
          cv::Mat aligned_image;
          image_in.copyTo(aligned_image);
          image_in = aligned_image;
        }
        QImage image(image_in.data, image_in.cols, image_in.rows, QImage::Format_RGB888);
        MainWindow::instance()->view()->update(image);
    } else {
        QImage image(&msg->data[0], msg->width, msg->height, QImage::Format_RGB888);
        MainWindow::instance()->view()->update(image);
    }
}

void Ros::publishCommand(std::string command){
  auto message = std_msgs::msg::String();
  message.data = command;
//   LOG("Publishing Request: %s", command.c_str());
}