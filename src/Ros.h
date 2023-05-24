#ifndef NODE_H
#define NODE_H

#include <QObject>
#include <memory>
#include <opencv2/videoio.hpp>
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
    void startRecording(std::string filename);
    void stopRecording(void);
public slots:
protected:
    void reelImageCall(const sensor_msgs::msg::Image::SharedPtr msg) ;
    void frontImageCall(const sensor_msgs::msg::Image::SharedPtr msg) const;
    void stateCallback(const std_msgs::msg::String::SharedPtr new_state) ;
    // void btCallback(const std_msgs::msg::String::SharedPtr new_state) ;
private:
    rclcpp::executors::StaticSingleThreadedExecutor::SharedPtr m_executor;
    rclcpp::Node::SharedPtr m_node;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_front_im;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_reel_im;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_state_update;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_bt_update;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_publisher;
    inline static std::list<IWatcher *> m_watchers; // inline defines it here:
    static Ros *s_self;
    bool m_recording;
    cv::VideoWriter m_writer_1;
};

#endif // NODE_H
