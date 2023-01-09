/*
author:jiangchao
date:2022.12.23
description:back charge server
*/

#include "back_charge_server/back_charge_server.h"

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace std;

/* This example creates a subclass of Node and uses std::bind() to control robot
 * back to charge */

namespace action_ChargeBack {

ChargeBack::ChargeBack(const rclcpp::NodeOptions &options)
    : Node("ChargeBack", options) {
  search_count = 1;
  info_get = false;

  // std::vector<std::string> new_args = rclcpp::NodeOptions().arguments();
  // new_args.push_back("--ros-args");
  // new_args.push_back("-r");
  // new_args.push_back(std::string("__node:=") + this->get_name() +
  //                    "_rclcpp_node");
  // new_args.push_back("--");
  // client_node_ = std::make_shared<rclcpp::Node>(
  //     "_", "", rclcpp::NodeOptions().arguments(new_args));
  twist_publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // robot_state_sub_ =
  // this->create_subscription<rcsbot_interface::msg::RobStatePub>("Robstate",
  //  qos, std::bind(&ChargeBack::state_callback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "backfunction comeback server");

  using namespace std::placeholders;

  this->back_action_server_ = rclcpp_action::create_server<ChargeBackAction>(
      this, "chargeback", std::bind(&ChargeBack::handle_goal, this, _1, _2),
      std::bind(&ChargeBack::handle_cancel, this, _1),
      std::bind(&ChargeBack::handle_accepted, this, _1));
}

void ChargeBack::state_callback(
    const rcsbot_interface::msg::RobStatePub::SharedPtr msg) {
  // std::unique_lock<std::mutex> lck(i_mutex);
  RCLCPP_INFO(this->get_logger(), "start state_callback");
  auto feedback = std::make_shared<ChargeBackAction::Feedback>();
  auto &current_status = feedback->current_status;
  auto result = std::make_shared<ChargeBackAction::Result>();
  // Check if asked to stop processing action
  if (is_cancel_requested()) {
    terminate_all(current_handle_);
    // terminate(current_handle_, result);
  }

  // Check if asked to process another action
  // //jc:是否有其他的任务需要做，比如更换导航点
  if (is_preempt_requested()) {
    RCLCPP_INFO(get_logger(), "Preempting the goal pose.");
    auto goal = accept_pending_goal(current_handle_);
    new_goal = true;
  }
  if (new_goal) {

    RCLCPP_INFO(this->get_logger(), "Executing back start");

    geometry_msgs::msg::Twist twist;
    // RCLCPP_INFO(this->get_logger(), "receive info: %d start to back ",
    // msg->m_chargeinfo); 先判断是否充上电
    if (msg->m_charged == true) {
      RCLCPP_INFO(this->get_logger(), "on charged");
      twist.linear.x = 0.0;
      twist.angular.z = 0.0;
      twist_publisher_->publish(twist);
      current_status = 0;
      current_handle_->publish_feedback(feedback);
      result->final_status = current_status;
      current_handle_->succeed(result);
      current_handle_.reset();
      new_goal = false;
      return;
    }
    int chargeInfo = msg->m_chargeinfo;
    static int num_rotate = 0;
    if (chargeInfo <= 0 || chargeInfo > 24) {
      current_status = 33;
      current_handle_->publish_feedback(feedback);
      info_get = false;
      RCLCPP_INFO(this->get_logger(),
                  "invalid chargeInfo %d start sway to get charge info",
                  chargeInfo);
      twist.linear.x = 0.0;
      if (search_count % 2 == 0) {
        twist.angular.z = (-1) * 0.05;
      } else {
        twist.angular.z = 0.05;
      }

      num_rotate++;
      if (num_rotate == STEP * search_count) {
        num_rotate = 0;
        search_count++;
      }
    }

    if (chargeInfo > 0 && chargeInfo <= 24) {
      current_status = 22;
      current_handle_->publish_feedback(feedback);
      info_get = true;
      num_rotate = 0;
      search_count = 1;
      // RCLCPP_INFO(this->get_logger(), "valid chargeInfo %d start pid
      // control", chargeInfo);
    }

    if (info_get) {
      twist = PidControl(chargeInfo);
    }

    RCLCPP_INFO(this->get_logger(), "cmd_vel:x %f cmd_vel:z %f chargeInfo: %d",
                twist.linear.x, twist.angular.z, chargeInfo);
    twist_publisher_->publish(twist);
  }
  // i_con.notify_one();
  // auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
} // namespace action_ChargeBack

void ChargeBack::moveback() {
  // std::unique_lock<std::mutex> lck(i_mutex);

  // RCLCPP_INFO(this->get_logger(), "Executing back start");

  RCLCPP_INFO(this->get_logger(), "Executing back charge");
}

geometry_msgs::msg::Twist ChargeBack::PidControl(int intput) {
  float error;
  geometry_msgs::msg::Twist twist;
  // static int pre_error = 0;
  // float derivative = 0;
  float output = 0;
  error = intput - TARGET;
  // derivative = error - pre_error;
  // pre_error = error;
  // output = kp * error + kd * derivative;
  output = kp * error * (-1);
  twist.linear.x = -0.05;
  twist.angular.z = output;
  return twist;
}

rclcpp_action::GoalResponse
ChargeBack::handle_goal(const rclcpp_action::GoalUUID &uuid,
                        std::shared_ptr<const ChargeBackAction::Goal> goal) {
  RCLCPP_INFO(this->get_logger(), "Received goal request ");
  (void)uuid;
  std::cout << goal << std::endl;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ChargeBack::handle_cancel(
    const std::shared_ptr<GoalHandleChargeBackAction> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ChargeBack::handle_accepted(
    const std::shared_ptr<GoalHandleChargeBackAction> handle) {
  auto qos_sensor = rclcpp::QoS(rclcpp::SensorDataQoS());
  // const auto goal = goal_handle->get_goal();
  // const auto goal = current_handle_->get_goal();
  // new_goal = true;
  robot_state_sub_ =
      this->create_subscription<rcsbot_interface::msg::RobStatePub>(
          "Robstate", qos_sensor,
          std::bind(&ChargeBack::state_callback, this, _1));

  std::lock_guard<std::recursive_mutex> lock(update_mutex_);
  RCLCPP_INFO(this->get_logger(), "Receiving a new goal");

  if (is_active(current_handle_)) {
    RCLCPP_INFO(
        this->get_logger(),
        "An older goal is active, moving the new goal to a pending slot.");

    if (is_active(pending_handle_)) {
      RCLCPP_INFO(
          this->get_logger(),
          "The pending slot is occupied."
          " The previous pending goal will be terminated and replaced.");
      terminate(pending_handle_);
    }
    pending_handle_ = handle;
    preempt_requested_ = true;
  } else {
    if (is_active(pending_handle_)) {
      // Shouldn't reach a state with a pending goal but no current one.
      RCLCPP_INFO(
          this->get_logger(),
          "Forgot to handle a preemption. Terminating the pending goal.");
      terminate(pending_handle_);
      preempt_requested_ = false;
    }

    current_handle_ = handle;

    // Return quickly to avoid blocking the executor, so spin up a new thread
    RCLCPP_INFO(this->get_logger(), "Executing goal asynchronously.");
    // execution_future_ = std::async(std::launch::async, [this]() { work(); });
  }
}

bool ChargeBack::is_cancel_requested() const {
  std::lock_guard<std::recursive_mutex> lock(update_mutex_);

  // A cancel request is assumed if either handle is canceled by the client.

  if (current_handle_ == nullptr) {
    RCLCPP_INFO(this->get_logger(),
                "Checking for cancel but current goal is not available");
    return false;
  }

  if (pending_handle_ != nullptr) {
    return pending_handle_->is_canceling();
  }

  return current_handle_->is_canceling();
}

ChargeBack::~ChargeBack() {}

} // namespace action_ChargeBack
