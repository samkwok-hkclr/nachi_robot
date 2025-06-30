#include "nachi_robot/nachi_robot_node.hpp"

NachiRobotNode::NachiRobotNode(const rclcpp::NodeOptions& options)
: Node("nachi_robot_node", options)
{
  bool use_fake_driver = false;

  declare_parameter<std::string>("model", "mz07L");
  get_parameter<std::string>("model", model_);

  declare_parameter<std::string>("ip", "192.168.0.214");
  get_parameter<std::string>("ip", ip_address_);
  
  declare_parameter<bool>("fake", true);
  get_parameter<bool>("fake", use_fake_driver);

  if (use_fake_driver)
    driver_ = std::make_shared<NachiFakeDriver>();
  else
    driver_ = std::make_shared<NachiDriver>();

  if (driver_->init(ip_address_, model_))
  {
    RCLCPP_INFO(get_logger(), "Init driver succeed!");
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "Init driver failed!");
    rclcpp::shutdown();
  }

  js_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  action_ser_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  js_timer_ = create_wall_timer(20ms, std::bind(&NachiRobotNode::joint_state_cb, this), js_cbg_);

  js_pub_ = create_publisher<JointState>("joint_states", 10);

  trajectory_action_ser_ = rclcpp_action::create_server<FollowJointTrajectory>(
    this,
    "mz07l_arm_controller/follow_joint_trajectory",
    std::bind(&NachiRobotNode::handle_goal, this, _1, _2),
    std::bind(&NachiRobotNode::handle_cancel, this, _1),
    std::bind(&NachiRobotNode::handle_accepted, this, _1),
    rcl_action_server_get_default_options(),
    action_ser_cbg_);
}

void NachiRobotNode::joint_state_cb(void)
{
  sensor_msgs::msg::JointState msg;

  if (driver_->read(msg))
  {
    msg.header.stamp = get_clock()->now();
    js_pub_->publish(msg);
    RCLCPP_DEBUG(get_logger(), "Nachi robot joint state published");
  }
}

rclcpp_action::GoalResponse NachiRobotNode::handle_goal(
  const rclcpp_action::GoalUUID& uuid, 
  std::shared_ptr<const FollowJointTrajectory::Goal> goal)
{
  (void)uuid;

  RCLCPP_INFO(get_logger(), "Received goal request with trajectory points %ld", goal->trajectory.points.size());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse NachiRobotNode::handle_cancel(
  const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
{
  (void)goal_handle;

  RCLCPP_INFO(get_logger(), "Received request to cancel goal");

  if (!driver_->cancel_trajectory())
    RCLCPP_ERROR(get_logger(), "Cancel goal failed ...");

  return rclcpp_action::CancelResponse::ACCEPT;
}

void NachiRobotNode::handle_accepted(
  const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
{
  std::thread{std::bind(&NachiRobotNode::execute, this, _1), goal_handle}.detach();
}

void NachiRobotNode::execute(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Executing goal");
  const auto goal = goal_handle->get_goal();

  driver_->execute_trajectory(goal->trajectory);


  if (rclcpp::ok())
  {
    auto result = std::make_shared<FollowJointTrajectory::Result>();

    result->error_code = result->SUCCESSFUL;
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Goal succeeded");
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<NachiRobotNode>(options);

  exec->add_node(node->get_node_base_interface());
  exec->spin();
    
  rclcpp::shutdown();
}
