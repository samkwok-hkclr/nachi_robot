#include "nachi_robot/nachi_driver.hpp"

NachiDriver::NachiDriver()
{
}

NachiDriver::~NachiDriver()
{
  if (nXmlOpenId > 0)
  {
    NR_Close(nXmlOpenId);
  }
}

bool NachiDriver::init(std::string ip_address, std::string model)
{
  if (model == "mz04")
  {
    joint_names.push_back("mz04_joint1");
    joint_names.push_back("mz04_joint2");
    joint_names.push_back("mz04_joint3");
    joint_names.push_back("mz04_joint4");
    joint_names.push_back("mz04_joint5");
    joint_names.push_back("mz04_joint6");
  }
  else if (model == "mz25")
  {
    joint_names.push_back("mz25_joint1");
    joint_names.push_back("mz25_joint2");
    joint_names.push_back("mz25_joint3");
    joint_names.push_back("mz25_joint4");
    joint_names.push_back("mz25_joint5");
    joint_names.push_back("mz25_joint6");
  }
  else if (model == "mz07L")
  {
    joint_names.push_back("joint_base_to_link1");
    joint_names.push_back("joint_link1_to_link2");
    joint_names.push_back("joint_link2_to_link3");
    joint_names.push_back("joint_link3_to_link4");
    joint_names.push_back("joint_link4_to_link5");
    joint_names.push_back("joint_link5_to_link6");
  }

  current_joint_position.push_back(0.0);
  current_joint_position.push_back(0.0);
  current_joint_position.push_back(0.0);
  current_joint_position.push_back(0.0);
  current_joint_position.push_back(0.1);
  current_joint_position.push_back(0.1);

  return connect(ip_address);
}

bool NachiDriver::connect(std::string ip_address)
{
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Connect to %s", ip_address.c_str());
  NACHI_COMMIF_INFO info;
  memset(&info, 0, sizeof(info));

  info.pcAddrs = &ip_address[0];
  info.lKind = NR_DATA_XML;

  nXmlOpenId = NR_Open(&info);
  if (0 < nXmlOpenId)
  {
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Established nXmlOpenId : %d", nXmlOpenId);
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Failed with error: %d", nXmlOpenId);
    return false;
  }

  rclcpp::sleep_for(std::chrono::milliseconds(1000));

  if (!release_motor())
    return false;

  return true;
}

bool NachiDriver::read(sensor_msgs::msg::JointState& msg)
{
  // Read joint states from robot using OpenNR
  if (get_joint_states())
  {
    // Return current joint position
    // msg.header.stamp = rclcpp::Time::now();
    msg.name = joint_names;
    msg.position = current_joint_position;
    return true;
  }

  return false;
}

bool NachiDriver::write(sensor_msgs::msg::JointState msg)
{
  if (msg.position.size() != joint_names.size())
  {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Joint position msg size %zu error", msg.position.size());
    return false;
  }

  return control_joint_position(msg.position, 0.01, false);
}

bool NachiDriver::control_joint_position(const std::vector<double>& js_cmd, float tol, bool accurate)
{
  if (js_cmd.size() == joint_names.size())
  {
    std::vector<double> js_temp(current_joint_position.begin(), current_joint_position.end());
    if (!set_joint_states(js_cmd, accurate))
    {
        return false;
    }

    int duration = 10;

    while (rclcpp::ok() && duration)
    {
      if (check_all_close(js_cmd, current_joint_position, tol))
        return true;
      
      rclcpp::sleep_for(std::chrono::milliseconds(100));

      if (check_all_close(js_temp, current_joint_position, 0.001))
      {
        if (!set_joint_states(js_cmd, accurate))
            return false;
        
        duration--;
      }
      else
      {
        for (int i = 0; i < (int)joint_names.size(); ++i)
        {
            js_temp[i] = current_joint_position[i];
        }
      }
    }
    
    RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME), "Execute joint states service call failed");
    return false;
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Joint position goal size %zu error", js_cmd.size());
  }
  
  return false;
}

bool NachiDriver::check_all_close(std::vector<double> a, std::vector<double> b, float tol)
{
  if (tol == 0)
    tol = 0.01;

  if (a.size() != b.size())
  {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Size mismatch error, %zu, %zu", a.size(), b.size());
    return true;
  }

  for (int i = 0; i < (int)a.size(); ++i)
  {
    float v_a = a[i];
    float v_b = b[i];

    if (std::abs(v_a - v_b) > tol)
    {
      //RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME),  _STREAM("Mismatch " << i << " " << v_a << " " << v_b);
      return false;
    }
  }
  return true;
}

bool NachiDriver::get_joint_states()
{
  if (nXmlOpenId < 0)
  {
    RCLCPP_DEBUG(rclcpp::get_logger(LOGGER_NAME), "No connection to robot (print every 11 seconds)");
    return false;
  }

  std::vector<float> curr_js_temp;
  // int axes_num = (int)joint_names.size();
  // float *fValue = new float[axes_num];
  // memset(fValue, 0, axes_num * sizeof(float));
  auto fValue = std::array<float, axes_num>{};

  int nErr = NR_AcsAxisTheta(nXmlOpenId, fValue.data(), 1, axes_num);
  if (NR_E_NORMAL == nErr)
  {
    for (int nAxis = 0; nAxis < (int)joint_names.size(); nAxis++)
    {
      //printf("Axis%d angle: %8.2f[deg]\n", (nAxis + 1), fValue[nAxis]);
      curr_js_temp.push_back(fValue[nAxis] * TO_RAD);
      if (nAxis == 1)
      {
          curr_js_temp[nAxis] -= M_PI_2;
      }
    }
    for (int i = 0; i < (int)joint_names.size(); i++)
    {
      current_joint_position[i] = curr_js_temp[i];
    }

    // delete[] fValue; 
    return true;
  }
  else
  {
    // delete[] fValue; 
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Read joint states error: %d", nErr);
    return false;
  }
}

bool NachiDriver::set_joint_states(std::vector<double> js_cmd, bool accurate)
{
  if (nXmlOpenId < 0)
    return false;

  // int axes_num = (int)joint_names.size();
  // float *fAngle = new float[axes_num];
  // memset(fAngle, 0, axes_num * sizeof(float));
  auto fAngle = std::array<float, axes_num>{};

  if ((int)js_cmd.size() != axes_num)
  {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Command size error: %zu", js_cmd.size());
    return false;
  }

  // The robot takes angle input in degree
  for (int i = 0; i < (int)joint_names.size(); i++)
  {
    fAngle[i] = static_cast<float>(js_cmd[i] * TO_DEG);
    // Here we offset the output value of axis 2
    if (i == 1)
    {
        fAngle[i] += 90.0;
    }
  }
  // NR_CtrlMoveJ: Move joint to absolute position
  // NR_CtrlMoveJA: Move joint to relative position
  int type;
  if (accurate)
    type = 1;
  else
    type = 0;

  int nErr = NR_CtrlMoveJ(nXmlOpenId, fAngle.data(), axes_num, type);
  // delete[] fAngle;
  if (nErr == NR_E_NORMAL)
  {
    return true;
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Write joint states error: %d", nErr);
    return false;
  }
}

float NachiDriver::get_speed_percentage()
{
  if (nXmlOpenId < 0)
    return false;
  
  float speed = 0.0;
  auto p = &speed;
  int nErr = NR_AcsSpeedOverride(nXmlOpenId, p, false);

  if (NR_E_NORMAL != nErr)
  {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Get speed Error: %d\n", nErr);
    return false;
  }        

  return speed;
}

bool NachiDriver::set_speed_percentage(float speed)
{
  if (nXmlOpenId < 0)
    return false;
  
  auto p = &speed;

  int nErr = NR_AcsSpeedOverride(nXmlOpenId, p, true);
  if (NR_E_NORMAL != nErr)
  {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Set speed Error: %d\n", nErr);
    return false;
  } 
  else 
  {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Set speed succeed to : %f\n", speed);
  }

  return true;
}

bool NachiDriver::release_motor()
{
  RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME), "Try Release Motor");

  for (int i = 0; i < RELEASE_MOTOR_RETIAL; ++i)
  {
    bool bValue[1];
    memset(bValue, 0, sizeof(bValue));

    int nErr = NR_AcsGeneralOutputSignal(nXmlOpenId, bValue, false, 201, 1);
    if (NR_E_NORMAL == nErr)
    {
      RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Reading Controller Output Signal Succeed, Ready Value : %d", bValue[0]);
      if (bValue[0])
      {
        int nErrCtrlMotor = NR_CtrlMotor(nXmlOpenId, 1);
        if (NR_E_NORMAL == nErrCtrlMotor)
        {
          RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME), "Motor Released Now");
          break;
        }
      }
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Reading Controller Ready Failed, Error code : %d", nErr);
    }        

    rclcpp::sleep_for(std::chrono::milliseconds(50));
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Failed Release Motor, Retry Count : %d", i);

    if (i >= RELEASE_MOTOR_RETIAL)
    {
      return false;
    }
  }

  return true;
}

bool NachiDriver::execute_trajectory(trajectory_msgs::msg::JointTrajectory trajectory)
{
  if (nXmlOpenId < 0)
    return false;

  int cnt = 0;

  for (int i = 0; i < (int)trajectory.points.size(); i++)
  {
    float fValue[6] = { static_cast<float>(trajectory.points[i].positions[0]) * TO_DEG,
                        static_cast<float>(trajectory.points[i].positions[1]) * TO_DEG + 90.0f,
                        static_cast<float>(trajectory.points[i].positions[2]) * TO_DEG,
                        static_cast<float>(trajectory.points[i].positions[3]) * TO_DEG,
                        static_cast<float>(trajectory.points[i].positions[4]) * TO_DEG,
                        static_cast<float>(trajectory.points[i].positions[5]) * TO_DEG };

    if ((cnt + 1) <= WRITE_BUF_SIZE)
    {
      NR_AcsGlobalFloat(nXmlOpenId, fValue, true, cnt * 6 + 1, 6);
    }
    else
    {
      NR_AcsGlobalFloat(nXmlOpenId, fValue, true, (cnt - WRITE_BUF_SIZE) * 6 + 301, 6);
      RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME), "Points #%d ", (cnt + 1));
    }

    cnt++;
  }
  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Points #%d ", cnt);

  if (cnt > (2 * WRITE_BUF_SIZE))
  {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Too many points");
    return false;
  }

  int num[1] = {cnt};
  int nErr = NR_AcsGlobalInt(nXmlOpenId, num, true, 200, 1);

  if (NR_E_NORMAL != nErr)
  {
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Write gloal int Error: %d\n", nErr);
  }

  if (!trajectory.header.frame_id.empty() && trajectory.header.frame_id.find_first_of("link") == std::string::npos)
  {
    float speed = std::stof(trajectory.header.frame_id);

    if (speed == 100.0)
    {
      set_speed_percentage(TOP_SPEED);  
    } 
    else 
    {
      set_speed_percentage(TOP_SPEED * speed / 100.0);
    }

    RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME), "Robot speed = %f", get_speed_percentage());
  }
  else
  {
    set_speed_percentage(TOP_SPEED/2);  
  }

  RCLCPP_WARN(rclcpp::get_logger(LOGGER_NAME), "Robot speed = %f", get_speed_percentage());

  int nErr1 = NR_CtrlProgram(nXmlOpenId, 1);
  if (NR_E_NORMAL != nErr1)
  {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "NR_CtrlProgram with error: %d\n", nErr1);
  }

  int nErr2 = NR_CtrlStep(nXmlOpenId, 0);
  if (NR_E_NORMAL != nErr2)
  {
      RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "NR_CtrlStep with error: %d\n", nErr2);
  }

  if (NR_E_NORMAL == nErr && NR_E_NORMAL == nErr1 && NR_E_NORMAL == nErr2)
  {
    int nErr3 = NR_CtrlRun(nXmlOpenId, 1);
    if (NR_E_NORMAL != nErr3)
    {
      RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "NR_CtrlRun with error: %d\n", nErr3);
    }
  }

  bool bValue = false;
  while (!bValue)
  {
    int nErr = NR_AcsFixedIOStartDisplay1(nXmlOpenId, &bValue);
    if (NR_E_NORMAL != nErr)
    {
      RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Monitor IO start OFF State, failed with error: %d\n", nErr);
    }
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }

  bValue = true;
  while (bValue)
  {
    int nErr = NR_AcsFixedIOStartDisplay1(nXmlOpenId, &bValue);
    if (NR_E_NORMAL != nErr)
    {
      RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Monitor IO start ON State, failed with error: %d\n", nErr);
    }
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }

  RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "Finished");

  return true;
}

bool NachiDriver::cancel_trajectory()
{
  int nErr = NR_CtrlRun(nXmlOpenId, 0);
  if (NR_E_NORMAL != nErr)
  {
    RCLCPP_ERROR(rclcpp::get_logger(LOGGER_NAME), "Cancel trajectory failed with error: %d\n", nErr);
    return false;
  }
  return true;     
}