#ifndef NODE_H
#define NODE_H

#include <QObject>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

#define LOG(...) RCLCPP_INFO(rclcpp::get_logger("image_subscriber"), __VA_ARGS__)

class Ros : public QObject {
    Q_OBJECT
public:
    static Ros *instance(void) { return s_self; }
    static void quit(void) { s_self->shutdown(); }
    Ros(int argc, char *argv[], const std::string &node_name);
    ~Ros();
    void publishCommand(std::string command);
    void spin(void);
    void spinOnBackground(void);
    void shutdown(void);
    rclcpp::Node::SharedPtr node(void) { return m_node; }
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher(void) {return m_publisher;};
public slots:
    void test() {std::cout << "hi" << std::endl;};
protected:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) const;
private:
    rclcpp::executors::StaticSingleThreadedExecutor::SharedPtr m_executor;
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_publisher;
    static Ros *s_self;
};

#endif // NODE_H
