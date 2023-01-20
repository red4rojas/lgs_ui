#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include "pipecrawler/action/crawleraction.hpp"
#include "pipecrawler/msg/crawlercmd.hpp"

#include "communicator.h"
#include "lgs_action_cli.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace lgs_ui;

void LGSActionClient::send_goal(signed short code){
    using namespace std::placeholders;

    if (!this->crawl_client_ptr_->wait_for_action_server(timeout)) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto crawler_goal = CrawlerAction::Goal();
    crawler_goal.command.pattern = {code};
    crawler_goal.command.looping = false;
    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<CrawlerAction>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&LGSActionClient::goal_response_callback, this, _1);
    // // send_goal_options.feedback_callback =
    // //   std::bind(&LGSActionClient::feedback_callback, this, _1, _2);
    // // send_goal_options.result_callback =
    // //   std::bind(&LGSActionClient::result_callback, this, _1);
    this->crawl_client_ptr_->async_send_goal(crawler_goal, send_goal_options);
  }

  LGSActionClient::LGSActionClient(const rclcpp::NodeOptions & options) : Node("lgs_action_client", options)  {
    this->crawl_client_ptr_ = rclcpp_action::create_client<CrawlerAction>(this,"crawler_action");
  }

  // Action Callbacks
  void LGSActionClient::goal_response_callback(std::shared_future<CrawlGoalHandle::SharedPtr> future)  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }
//   void feedback_callback(
//     CrawlGoalHandle::SharedPtr,
//     const std::shared_ptr<const CrawlAction::Feedback> feedback)
//   {
//     std::stringstream ss;
//     ss << "Next number in sequence received: ";
//     for (auto number : feedback->partial_sequence) {
//       ss << number << " ";
//     }
//     RCLCPP_INFO(this->get_logger(), ss.str().c_str());
//   }

//   void result_callback(const CrawlGoalHandle::WrappedResult & result)
//   {
//     switch (result.code) {
//       case rclcpp_action::ResultCode::SUCCEEDED:
//         break;
//       case rclcpp_action::ResultCode::ABORTED:
//         RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
//         return;
//       case rclcpp_action::ResultCode::CANCELED:
//         RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
//         return;
//       default:
//         RCLCPP_ERROR(this->get_logger(), "Unknown result code");
//         return;
//     }
    // std::stringstream ss;
    // ss << "Result received: ";
    // for (auto number : result.result->sequence) {
    //   ss << number << " ";
    // }
//     RCLCPP_INFO(this->get_logger(), ss.str().c_str());
//     rclcpp::shutdown();
//   }

LGSActionClient * LGSActionClient::instance_ = nullptr; 

lgs_ui::LGSActionClient* lgs_ui::LGSActionClient::GetInstance(const rclcpp::NodeOptions & options){
    if (instance_ == nullptr){
      instance_ = new LGSActionClient(options); 
    } 
      return instance_;
}