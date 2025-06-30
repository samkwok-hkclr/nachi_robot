#include "nachi_robot/nachi_fake_driver.hpp"

NachiFakeDriver::NachiFakeDriver()
{
}

NachiFakeDriver::~NachiFakeDriver()
{
}

bool NachiFakeDriver::connect(std::string ip_address)
{
  (void) ip_address;
  return true;
}

bool NachiFakeDriver::read(sensor_msgs::msg::JointState& msg)
{
  // Read joint states from robot using OpenNR
  if (get_joint_states())
  {
    // Return current joint position
    msg.name = joint_names;
    std::vector<double> q;
    q.reserve(joint_names.size());

    for (int i = 0; i < (int)joint_names.size(); i++)
    {
      q.push_back(current_joint_position[i]);
    }

    msg.position = q;
    return true;
  }

  return false;
}

bool NachiFakeDriver::write(sensor_msgs::msg::JointState msg)
{
  if (msg.position.size() != joint_names.size())
  {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Joint position msg size %zu error", msg.position.size());
    return false;
  }

  return set_joint_states(msg.position, true);
}

bool NachiFakeDriver::get_joint_states()
{
  return true;
}

bool NachiFakeDriver::set_joint_states(std::vector<double> js_cmd, bool accurate)
{
  (void)accurate;

  bool update = false;
  for (int i = 0; i < (int)joint_names.size(); i++)
  {
    if (js_cmd[i] != current_joint_position[i])
    {
      update = true;
      break;
    }
  }

  if (update)
  {
    current_joint_position = js_cmd;
    i++; // what is variable i?
  }

  return true;
}

bool NachiFakeDriver::execute_trajectory(trajectory_msgs::msg::JointTrajectory trajectory)
{
  rclcpp::Rate rate(20 * sim_time_scale);

  for (int i = 0; i < (int)trajectory.points.size(); i++)
  {
    set_joint_states(trajectory.points[i].positions, true);
    rate.sleep();
  }

  return true;
}