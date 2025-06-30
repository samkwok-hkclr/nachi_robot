#ifndef NACHI_FAKE_DRIVER_HPP__
#define NACHI_FAKE_DRIVER_HPP__

#include <cmath>
#include <cstdio>
#include <mutex>

#include "rclcpp/rclcpp.hpp"

#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include "nachi_robot/nachi_driver.hpp"
#include "nachi_robot/nachi_fake_driver.hpp"

class NachiFakeDriver : public NachiDriver
{
public:
  NachiFakeDriver();
  ~NachiFakeDriver();

  bool read(sensor_msgs::msg::JointState& msg) override;
  bool write(sensor_msgs::msg::JointState msg) override;

  bool execute_trajectory(const trajectory_msgs::msg::JointTrajectory trjectory) override;

  void set_sim_time_scale(double sim_time_scale) 
  {
    this->sim_time_scale = sim_time_scale;
  }

protected:
  // unknown variable form AutoStore source code
  int i = 0;

  double sim_time_scale = 1.0;

  bool connect(std::string ip_address) override;

  bool get_joint_states() override;
  bool set_joint_states(std::vector<double> js_cmd, bool accurate) override;
};

#endif