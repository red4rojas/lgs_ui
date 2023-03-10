#include <functional>
#include <future>
#include <vector>
#include <memory>
#include <string>
#include <sstream>
#include "pipecrawler/action/crawleraction.hpp"
#include "reel/action/reelaction.hpp"
#include "communicator.h"
#include "lgs_action_cli.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace lgs_ui;
using namespace std::placeholders;
void LGSActionClient::call_crawler(signed short code){
    this->wait_servers();
    auto crawler_goal = CrawlerAction::Goal();
    crawler_goal.command.pattern = {code};
    crawler_goal.command.looping = false;
    auto send_goal_options = rclcpp_action::Client<CrawlerAction>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&LGSActionClient::goal_response_callback, this, _1);
    // send_goal_options.feedback_callback = std::bind(&LGSActionClient::feedback_callback, this, _1, _2);
    // send_goal_options.result_callback = std::bind(&LGSActionClient::result_callback, this, _1);
    this->crawl_client_ptr_->async_send_goal(crawler_goal, send_goal_options);
}

void LGSActionClient::call_crawler(std::vector<signed short> pattern){
    this->wait_servers();
    auto crawler_goal = CrawlerAction::Goal();
    crawler_goal.command.pattern = rosidl_runtime_cpp::BoundedVector<int16_t, 6UL, std::allocator<int16_t>>(
      pattern.begin(), pattern.end()
    );
    if (pattern[0] == 0){
      crawler_goal.command.looping = false;
    } else {
      crawler_goal.command.looping = true;
    }
    auto send_goal_options = rclcpp_action::Client<CrawlerAction>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&LGSActionClient::goal_response_callback, this, _1);
    // send_goal_options.feedback_callback = std::bind(&LGSActionClient::feedback_callback, this, _1, _2);
    // send_goal_options.result_callback = std::bind(&LGSActionClient::result_callback, this, _1);
    this->crawl_client_ptr_->async_send_goal(crawler_goal, send_goal_options);
}

void LGSActionClient::call_reel(int vel, float interval, bool continuous){
  auto reel_goal = ReelAction::Goal();
  reel_goal.reelcommand.reel_vel = vel;
  reel_goal.reelcommand.interval = interval;
  reel_goal.reelcommand.continuous = continuous;
  auto send_goal_options = rclcpp_action::Client<ReelAction>::SendGoalOptions();
  this->reel_client_ptr_->async_send_goal(reel_goal, send_goal_options);
}

LGSActionClient::LGSActionClient(const rclcpp::NodeOptions & options) : Node("lgs_action_client", options)  {
  this->crawl_client_ptr_ = rclcpp_action::create_client<CrawlerAction>(this,"crawler_action"); 
  this->reel_client_ptr_ = rclcpp_action::create_client<ReelAction>(this,"activate_reel"); 
}

void LGSActionClient::goal_response_callback(std::shared_future<CrawlGoalHandle::SharedPtr> future)  {
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

//  void feedback_callback(
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

void LGSActionClient::pass_command(std::string command){
  RCLCPP_INFO(this->get_logger(), "Sending goal: %s", command.c_str());
  if (command == "front_grip_on"){
    call_crawler(1);
  } else if (command == "front_grip_off"){
    call_crawler(2); 
  } else if (command == "extender_on"){
    call_crawler(3);
  } else if (command == "extender_off"){
    call_crawler(4);
  } else if (command == "back_grip_on"){
    call_crawler(5);
  } else if (command == "back_grip_off"){
    call_crawler(6);   
  } else if (command == "forward"){
    call_crawler(std::vector<signed short>({1,3,5,2,4,6}));
    call_reel(1,2.22,true);   
  } else if (command == "backward"){
    // call_crawler(std::vector<signed short>({5,3,1,6,4,2}));
    call_crawler(std::vector<signed short>({2,4,6}));
    call_reel(-2,0.8,true);   
  } else if (command == "stop"){
    call_crawler(std::vector<signed short>({0,0,0,0,0,0}));
    call_reel(0,0,false);   
  } else {
    RCLCPP_INFO(this->get_logger(), "Ops, invalid command");
  }
}

void LGSActionClient::wait_servers(){
    if (!this->crawl_client_ptr_->wait_for_action_server(timeout)) {
    RCLCPP_ERROR(this->get_logger(), "Crawler server not available after waiting");
    rclcpp::shutdown();
  }

  if (!this->reel_client_ptr_->wait_for_action_server(timeout)) {
    RCLCPP_ERROR(this->get_logger(), "Reel server not available after waiting");
    rclcpp::shutdown();
  }
}

LGSActionClient * LGSActionClient::instance_ = nullptr; 

lgs_ui::LGSActionClient* lgs_ui::LGSActionClient::GetInstance(const rclcpp::NodeOptions & options){
  if (instance_ == nullptr){
    instance_ = new LGSActionClient(options); 
  } 
    return instance_;
}