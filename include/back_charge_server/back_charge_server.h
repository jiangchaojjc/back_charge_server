//
// Created by JC on 2022/11/21
//

#ifndef CHARGEBACK_H
#define CHARGEBACK_H

#include "std_msgs/msg/string.hpp"
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <thread> // std::this_thread::sleep_for

#include "rclcpp/rclcpp.hpp"
#include <chrono>

#include <functional>
#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <tf2_ros/transform_broadcaster.h>

// Add Custom Message
#include "rcsbot_interface/msg/error_pub.hpp"
#include "rcsbot_interface/msg/io_pub.hpp"
#include "rcsbot_interface/msg/rob_state_pub.hpp"
#include "rcsbot_interface/msg/sonar_pub.hpp"

#include "charge_interface/action/charge_back.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "back_charge_server/visibility_control.h"

#define TARGET 12
#define kp 0.05
#define ki 0
#define kd 0.05
#define STEP 20

namespace action_ChargeBack {

class ChargeBack : public rclcpp::Node {
public:
  using ChargeBackAction = charge_interface::action::ChargeBack;
  using GoalHandleChargeBackAction =
      rclcpp_action::ServerGoalHandle<ChargeBackAction>;
  // ACTION_TUTORIALS_CPP_PUBLIC
  explicit ChargeBack(const rclcpp::NodeOptions &options);
  ~ChargeBack();
  // void state_callback();

  template <typename ActionT>
  const std::shared_ptr<const typename ActionT::Goal>
  accept_pending_goal(std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>
                          current_handle_) {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);

    if (!pending_handle_ || !pending_handle_->is_active()) {
      RCLCPP_INFO(this->get_logger(),
                  "Attempting to get pending goal when not available");
      return std::shared_ptr<const typename ActionT::Goal>();
    }

    if (is_active(current_handle_) && current_handle_ != pending_handle_) {
      RCLCPP_INFO(this->get_logger(), "Cancelling the previous goal");
      current_handle_->abort(empty_result(current_handle_));
    }

    current_handle_ = pending_handle_;
    pending_handle_.reset();
    preempt_requested_ = false;

    RCLCPP_INFO(this->get_logger(), "Preempted goal");

    return current_handle_->get_goal();
  }

  template <typename ActionT>
  void terminate_all(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> handle,
      typename std::shared_ptr<typename ActionT::Result> result =
          std::make_shared<typename ActionT::Result>()) {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    terminate(current_handle_, result);
    terminate(pending_handle_, result);
    preempt_requested_ = false;
  }

protected:
  std::shared_ptr<rclcpp_action::ServerGoalHandle<ChargeBackAction>>
      current_handle_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<ChargeBackAction>>
      pending_handle_;
  rclcpp::Node::SharedPtr client_node_;
  mutable std::recursive_mutex update_mutex_;
  bool preempt_requested_{false};

  template <typename ActionT>
  void
  terminate(std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> handle,
            typename std::shared_ptr<typename ActionT::Result> result =
                std::make_shared<typename ActionT::Result>()) {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);

    if (handle != nullptr && handle->is_active()) {
      if (handle->is_canceling()) {
        RCLCPP_INFO(this->get_logger(),
                    "Client requested to cancel the goal. Cancelling.");
        handle->canceled(result);
      } else {
        RCLCPP_INFO(this->get_logger(), "Aborting handle.");
        handle->abort(result);
      }
      handle.reset();
    }
  }

  // std::shared_ptr<ChargeBackAction::Result> empty_result() const {
  //   return std::make_shared<ChargeBackAction::Result>();
  // }

  template <typename ActionT>
  constexpr auto empty_result(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> handle) const {
    return std::make_shared<typename ActionT::Result>();
  }

  template <typename ActionT>
  constexpr bool is_active(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> handle)
      const {
    return handle != nullptr && handle->is_active();
  }

  bool is_preempt_requested() const {
    std::lock_guard<std::recursive_mutex> lock(update_mutex_);
    return preempt_requested_;
  }

private:
  void state_callback(const rcsbot_interface::msg::RobStatePub::SharedPtr msg);
  void moveback();
  // void state_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  geometry_msgs::msg::Twist PidControl(int intput);

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const ChargeBackAction::Goal> goal);

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleChargeBackAction> goal_handle);

  void
  handle_accepted(const std::shared_ptr<GoalHandleChargeBackAction> handle);

  bool is_cancel_requested() const;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
  rclcpp::Subscription<rcsbot_interface::msg::RobStatePub>::SharedPtr
      robot_state_sub_;
  // rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
  // robot_state_sub_;
  int search_count;
  bool info_get;
  rclcpp_action::Server<ChargeBackAction>::SharedPtr back_action_server_;
  int nav2_goal_get;
  rcsbot_interface::msg::RobStatePub::SharedPtr state_msg;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex i_mutex;
  std::condition_variable i_con;
  bool new_goal;
};
} // namespace action_ChargeBack

RCLCPP_COMPONENTS_REGISTER_NODE(action_ChargeBack::ChargeBack)
#endif // CHARGEBACK_H
