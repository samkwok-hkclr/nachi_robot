#ifndef NACHI_DRIVER_HPP__
#define NACHI_DRIVER_HPP__

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

// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Matrix3x3.h>
// #include <tf2/LinearMath/Scalar.h>

#include "nachi_header/OpenNR-IF.h"

#define LOGGER_NAME (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

class NachiDriver
{
public:
  NachiDriver();
  ~NachiDriver();

  // bool init(std::string ip_address, int gripper_port, int converyor_port, std::string model);
  bool init(std::string ip_address, std::string model);

  virtual bool read(sensor_msgs::msg::JointState& msg);
  virtual bool write(sensor_msgs::msg::JointState msg);

  // It should be used in the AutoStore V1 variable
  // virtual bool controlGripper(bool enable);
  // virtual bool getConveyorState(bool& state);
  
  virtual bool execute_trajectory(const trajectory_msgs::msg::JointTrajectory trjectory);
  bool cancel_trajectory();

  float get_speed_percentage();
  bool set_speed_percentage(float speed);

protected:
  int nXmlOpenId;

  // It should be used in the AutoStore V1 variable
  // int gripperPort;
  // int conveyorPort;

  static const int axes_num = 6;

  std::vector<std::string> joint_names;
  std::vector<double> current_joint_position;

  const float TO_METER = 0.001;
  const float TO_MM = 1000.0;

  const float TO_DEG = 180. / M_PI;
  const float TO_RAD = M_PI / 180.0;

  const int WRITE_BUF_SIZE = 33;

  const float TOP_SPEED = 100.0;

  const int RELEASE_MOTOR_RETIAL = 20;

  virtual bool connect(std::string ip_address);

  virtual bool get_joint_states();
  virtual bool set_joint_states(std::vector<double> js_cmd, bool accurate);

  bool control_joint_position(const std::vector<double>& js_cmd, float tol, bool accurate);
  bool check_all_close(std::vector<double> a, std::vector<double> b, float tol);

  bool release_motor();
};

#endif