#ifndef NACHI_ROBOT_NODE_HPP__
#define NACHI_ROBOT_NODE_HPP__

#include <functional>
#include <future>
#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>

#include "sensor_msgs/msg/joint_state.hpp"

#include "nachi_robot/nachi_driver.hpp"
#include "nachi_robot/nachi_fake_driver.hpp"

using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class NachiRobotNode : public rclcpp::Node
{
  using JointState = sensor_msgs::msg::JointState;
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFollowJointTrajectory = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

public:
  explicit NachiRobotNode(const rclcpp::NodeOptions& options);
  ~NachiRobotNode() = default;
  
  bool init(std::string ip_address, int gripper_port, int converyor_port, std::string model);
  
  void joint_state_cb(void);

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);

  void execute(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);

private:
  std::string model_;
  std::string ip_address_;

  std::shared_ptr<NachiDriver> driver_;

  rclcpp::CallbackGroup::SharedPtr js_cbg_;
  rclcpp::CallbackGroup::SharedPtr action_ser_cbg_;
  rclcpp::TimerBase::SharedPtr js_timer_;

  rclcpp::Publisher<JointState>::SharedPtr js_pub_;

  // rclcpp::Subscription<???>::SharedPtr ???_sub_;
  // rclcpp::Service<???>::SharedPtr ???_srv_;

  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr trajectory_action_ser_;
};

#endif // NACHI_ROBOT_NODE_HPP__