#ifndef NODE_H
#define NODE_H

#include <QObject>
#include <memory>
#include "lgs_interfaces/msg/crawlerstate.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include "components/IWatcher.h"

#define LOG(...) RCLCPP_INFO(rclcpp::get_logger("image_subscriber"), __VA_ARGS__)

class Ros : public QObject {
    Q_OBJECT
public:
    Ros(int argc, char *argv[], const std::string &node_name);
    ~Ros();
    static Ros *instance(void) { return s_self; }
    static void quit(void) { s_self->shutdown(); }
    static void addWatcher(IWatcher * new_watcher);
    void publishCommand(std::string command);
    void spin(void);
    void spinOnBackground(void);
    void shutdown(void);
    rclcpp::Node::SharedPtr node(void) { return m_node; }
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher(void) {return m_publisher;};
public slots:
protected:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) const;
    void stateCallback(const lgs_interfaces::msg::Crawlerstate::SharedPtr new_state) const;
private:
    rclcpp::executors::StaticSingleThreadedExecutor::SharedPtr m_executor;
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image;
    rclcpp::Subscription<lgs_interfaces::msg::Crawlerstate>::SharedPtr m_state_update;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_publisher;
    inline static std::list<IWatcher *> m_watchers; // inline defines it here:
    static Ros *s_self;
};

#endif // NODE_H
