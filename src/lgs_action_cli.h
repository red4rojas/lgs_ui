#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <vector>
#include "pipecrawler/action/crawleraction.hpp"
#include "reel/action/reelaction.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#ifndef LGSCLI_H
#define LGSCLI_H

namespace lgs_ui
{
class LGSClient : public rclcpp::Node
{
public:
  using CrawlerAction = pipecrawler::action::Crawleraction;
  using CrawlGoalHandle = rclcpp_action::ClientGoalHandle<CrawlerAction>;
  using ReelAction = reel::action::Reelaction;
  using ReelGoalHandle = rclcpp_action::ClientGoalHandle<ReelAction>;

  LGSClient(LGSClient &other) = delete;
  void operator=(const LGSClient &) = delete;
  void wait_servers();
  void pass_command(std::string command);

  static LGSClient* GetInstance(const rclcpp::NodeOptions & options);
  static LGSClient * instance_;

private:
  void call_crawler(signed short code);
  void call_crawler(std::vector<signed short> pattern); 
  void call_reel(int vel, float interval, bool continous);
  rclcpp_action::Client<CrawlerAction>::SharedPtr crawl_client_ptr_;
  rclcpp_action::Client<ReelAction>::SharedPtr reel_client_ptr_;
  rclcpp_action::Server<Std_msgs::String>::SharedPtr reel_client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::milliseconds timeout = std::chrono::milliseconds(1000);

  explicit LGSClient(const rclcpp::NodeOptions & options);

  void goal_response_callback(std::shared_future<CrawlGoalHandle::SharedPtr> future);
//   void feedback_callback()
//   void result_callback(const CrawlGoalHandle::WrappedResult & result)
};
}
// RCLCPP_COMPONENTS_REGISTER_NODE(lgs_ui::LGSClient)
#endif