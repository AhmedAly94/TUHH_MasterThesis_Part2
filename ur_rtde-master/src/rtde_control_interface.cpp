#include <ur_rtde/dashboard_client.h>
#include <ur_rtde/robot_state.h>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/script_client.h>
#if !defined(_WIN32) && !defined(__APPLE__)
#include <urcl/script_sender.h>
#endif

#include <ur_rtde/rtde_utility.h>

#include <bitset>
#include <boost/thread/thread.hpp>
#include <chrono>
#include <functional>
#include <iostream>
#include <thread>

namespace ur_rtde
{
static const std::string move_path_inject_id = "# inject move path\n";

static void verifyValueIsWithin(const double &value, const double &min, const double &max)
{
  if (std::isnan(min) || std::isnan(max))
  {
    throw std::invalid_argument("Make sure both min and max are not NaN's");
  }
  else if (std::isnan(value))
  {
    throw std::invalid_argument("The value is considered NaN");
  }
  else if (!(std::isgreaterequal(value, min) && std::islessequal(value, max)))
  {
    std::ostringstream oss;
    oss << "The value is not within [" << min << ";" << max << "]";
    throw std::range_error(oss.str());
  }
}

RTDEControlInterface::RTDEControlInterface(std::string hostname, uint16_t flags, int ur_cap_port)
    : hostname_(std::move(hostname)),
      upload_script_(flags & FLAG_UPLOAD_SCRIPT),
      use_external_control_ur_cap_(flags & FLAG_USE_EXT_UR_CAP),
      verbose_(flags & FLAG_VERBOSE),
      use_upper_range_registers_(flags & FLAG_UPPER_RANGE_REGISTERS),
      no_wait_(flags & FLAG_NO_WAIT),
      custom_script_(flags & FLAG_CUSTOM_SCRIPT),
      ur_cap_port_(ur_cap_port)
{
  // Create a connection to the dashboard server
  db_client_ = std::make_shared<DashboardClient>(hostname_);
  db_client_->connect();
  PolyScopeVersion polyscope_version(db_client_->polyscopeVersion());
  if (polyscope_version.major == 5 && polyscope_version.minor > 5)
  {
    serial_number_ = db_client_->getSerialNumber();
  }

  // Only check if in remote on real robot or when not using the ExternalControl UR Cap.
  if (!use_external_control_ur_cap_)
  {
    // 192.168.56.101 is the CI ursim test ip address.
    if (hostname_ != "localhost" && hostname_ != "127.0.0.1" && hostname_ != "192.168.56.101")
    {
      if (polyscope_version.major == 5 && polyscope_version.minor > 5)
      {
        // Check if robot is in remote control
        if (!db_client_->isInRemoteControl())
        {
          throw std::logic_error("ur_rtde: Please enable remote control on the robot!");
        }
      }
    }
  }
  port_ = 30004;
  custom_script_running_ = false;
  rtde_ = std::make_shared<RTDE>(hostname_, port_, verbose_);
  rtde_->connect();
  rtde_->negotiateProtocolVersion();
  versions_ = rtde_->getControllerVersion();

  frequency_ = 125;
  // If e-Series Robot set frequency to 500Hz
  if (versions_.major > CB3_MAJOR_VERSION)
    frequency_ = 500;

  // Set delta time to be used by receiveCallback
  delta_time_ = 1 / frequency_;

  // Create a connection to the script server
  script_client_ = std::make_shared<ScriptClient>(hostname_, versions_.major,
                                                  versions_.minor);
  script_client_->connect();

  // If user want to use upper range of RTDE registers, add the register offset in control script
  if (use_upper_range_registers_)
  {
    script_client_->setScriptInjection("# float register offset\n", "24");
    script_client_->setScriptInjection("# int register offset\n", "24");
    register_offset_ = 24;
  }
  else
  {
    script_client_->setScriptInjection("# float register offset\n", "0");
    script_client_->setScriptInjection("# int register offset\n", "0");
    register_offset_ = 0;
  }

  // Setup default recipes
  setupRecipes(frequency_);

  // Init Robot state
  robot_state_ = std::make_shared<RobotState>(state_names_);

  // Wait until RTDE data synchronization has started
  if (verbose_)
    std::cout << "Waiting for RTDE data synchronization to start..." << std::endl;
  std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();

  // Start RTDE data synchronization
  rtde_->sendStart();

  while (!rtde_->isStarted())
  {
    // Wait until RTDE data synchronization has started or timeout
    std::chrono::high_resolution_clock::time_point current_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
    if (duration > RTDE_START_SYNCHRONIZATION_TIMEOUT)
    {
      break;
    }
  }

  if (!rtde_->isStarted())
    throw std::logic_error("Failed to start RTDE data synchronization, before timeout");

  // Start executing receiveCallback
  th_ = std::make_shared<boost::thread>(boost::bind(&RTDEControlInterface::receiveCallback, this));

  // Wait until the first robot state has been received
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // Clear command register
  sendClearCommand();

  if (upload_script_)
  {
    if (!isProgramRunning())
    {
      // Send script to the UR Controller
      script_client_->sendScript();
      waitForProgramRunning();
    }
    else
    {
      if (verbose_)
        std::cout << "A script was running on the controller, killing it!" << std::endl;
      // Stop the running script first
      stopScript();
      db_client_->stop();

      // Wait until terminated
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

      // Send script to the UR Controller
      script_client_->sendScript();

      while (!isProgramRunning())
      {
        // Wait for program to be running
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
  }

#if !defined(_WIN32) && !defined(__APPLE__)
  // When the user wants to use ur_rtde with the ExternalControl UR Cap
  if (!upload_script_ && use_external_control_ur_cap_)
  {
    // Create a connection to the ExternalControl UR cap for sending scripts to the cap
    urcl_script_sender_.reset(new urcl::comm::ScriptSender(ur_cap_port_, script_client_->getScript(), false));
    urcl_script_sender_->start();

    if (!no_wait_)
    {
      if (!isProgramRunning())
      {
        start_time = std::chrono::high_resolution_clock::now();
        std::cout << "Waiting for RTDE control program to be running on the controller" << std::endl;
        while (!isProgramRunning())
        {
          std::chrono::high_resolution_clock::time_point current_time = std::chrono::high_resolution_clock::now();
          auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
          if (duration > WAIT_FOR_PROGRAM_RUNNING_TIMEOUT)
          {
            break;
          }
          // Wait for program to be running
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        if (!isProgramRunning())
        {
          disconnect();
          throw std::logic_error("RTDE control program is not running on controller, before timeout of " +
                                 std::to_string(WAIT_FOR_PROGRAM_RUNNING_TIMEOUT) + " seconds");
        }
      }
    }
  }
#else
  if (!upload_script_ && use_external_control_ur_cap_)
  {
    throw std::logic_error("The use of ExternalControl UR Cap is not supported on Windows and Apple yet."
        " Please contact author");
  }
#endif

  // When the user wants to a custom script / program on the controller interacting with ur_rtde.
  if (!upload_script_ && !use_external_control_ur_cap_)
  {
    if (!no_wait_)
    {
      if (!isProgramRunning())
      {
        start_time = std::chrono::high_resolution_clock::now();
        std::cout << "Waiting for RTDE control program to be running on the controller" << std::endl;
        while (!isProgramRunning())
        {
          std::chrono::high_resolution_clock::time_point current_time = std::chrono::high_resolution_clock::now();
          auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
          if (duration > WAIT_FOR_PROGRAM_RUNNING_TIMEOUT)
          {
            break;
          }
          // Wait for program to be running
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        if (!isProgramRunning())
        {
          disconnect();
          throw std::logic_error("RTDE control program is not running on controller, before timeout of " +
                                 std::to_string(WAIT_FOR_PROGRAM_RUNNING_TIMEOUT) + " seconds");
        }
      }
    }
  }
}

RTDEControlInterface::~RTDEControlInterface()
{
  disconnect();
}

void RTDEControlInterface::waitForProgramRunning()
{
  int ms_count = 0;
  int ms_retry_count = 0;
  while (!isProgramRunning())
  {
    // Wait for program to be running
    static const int sleep_ms = 10;
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
    ms_count += sleep_ms;
    ms_retry_count += sleep_ms;
    if (ms_retry_count >= 400)
    {
      ms_retry_count = 0;
      if (verbose_)
        std::cout << "ur_rtde: Program not running - resending script" << std::endl;
      script_client_->sendScript();
    }
    if (ms_count > 5000)
    {
      throw std::logic_error("ur_rtde: Failed to start control script, before timeout");
    }
  }
}

void RTDEControlInterface::disconnect()
{
  // Stop the receive callback function
  stop_thread_ = true;
  th_->interrupt();
  th_->join();

  if (rtde_ != nullptr)
  {
    if (rtde_->isConnected())
      rtde_->disconnect();
  }

  if (script_client_ != nullptr)
  {
    if (script_client_->isConnected())
      script_client_->disconnect();
  }

  if (db_client_ != nullptr)
  {
    if (db_client_->isConnected()) {
      db_client_->disconnect();
      serial_number_.clear();
    }
  }

  // Wait until everything has disconnected
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

bool RTDEControlInterface::isConnected()
{
  return rtde_->isConnected();
}

bool RTDEControlInterface::reconnect()
{
  db_client_->connect();
  PolyScopeVersion polyscope_version(db_client_->polyscopeVersion());
  if (polyscope_version.major == 5 && polyscope_version.minor > 5)
  {
    serial_number_ = db_client_->getSerialNumber();
  }
  script_client_->connect();
  rtde_->connect();
  rtde_->negotiateProtocolVersion();
  versions_ = rtde_->getControllerVersion();

  frequency_ = 125;
  // If e-Series Robot set frequency to 500Hz
  if (versions_.major > CB3_MAJOR_VERSION)
    frequency_ = 500;

  // Set delta time to be used by receiveCallback
  delta_time_ = 1 / frequency_;

  // If user want to use upper range of RTDE registers, add the register offset in control script
  if (use_upper_range_registers_)
  {
    script_client_->setScriptInjection("# float register offset\n", "24");
    script_client_->setScriptInjection("# int register offset\n", "24");
    register_offset_ = 24;
  }
  else
  {
    script_client_->setScriptInjection("# float register offset\n", "0");
    script_client_->setScriptInjection("# int register offset\n", "0");
    register_offset_ = 0;
  }

  // Setup default recipes
  setupRecipes(frequency_);

  // Init Robot state
  robot_state_ = std::make_shared<RobotState>(state_names_);

  // Wait until RTDE data synchronization has started.
  if (verbose_)
    std::cout << "Waiting for RTDE data synchronization to start..." << std::endl;
  std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();

  // Start RTDE data synchronization
  rtde_->sendStart();

  while (!rtde_->isStarted())
  {
    // Wait until RTDE data synchronization has started or timeout
    std::chrono::high_resolution_clock::time_point current_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
    if (duration > RTDE_START_SYNCHRONIZATION_TIMEOUT)
    {
      break;
    }
  }

  if (!rtde_->isStarted())
    throw std::logic_error("Failed to start RTDE data synchronization, before timeout");

  // Start executing receiveCallback
  stop_thread_ = false;
  th_ = std::make_shared<boost::thread>(boost::bind(&RTDEControlInterface::receiveCallback, this));

  // Wait until the first robot state has been received
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // Clear command register
  sendClearCommand();

  if (upload_script_)
  {
    if (!isProgramRunning())
    {
      // Send script to the UR Controller
      script_client_->sendScript();
      waitForProgramRunning();
    }
    else
    {
      if (verbose_)
        std::cout << "A script was running on the controller, killing it!" << std::endl;
      // Stop the running script first
      stopScript();
      db_client_->stop();

      // Wait until terminated
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

      // Send script to the UR Controller
      script_client_->sendScript();

      while (!isProgramRunning())
      {
        // Wait for program to be running
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
  }

#if !defined(_WIN32) && !defined(__APPLE__)
  // When the user wants to use ur_rtde with the ExternalControl UR Cap
  if (!upload_script_ && use_external_control_ur_cap_)
  {
    // Create a connection to the ExternalControl UR cap for sending scripts to the cap
    urcl_script_sender_.reset(new urcl::comm::ScriptSender(ur_cap_port_, script_client_->getScript(), false));
    urcl_script_sender_->start();

    if (!no_wait_)
    {
      if (!isProgramRunning())
      {
        start_time = std::chrono::high_resolution_clock::now();
        std::cout << "Waiting for RTDE control program to be running on the controller" << std::endl;
        while (!isProgramRunning())
        {
          std::chrono::high_resolution_clock::time_point current_time = std::chrono::high_resolution_clock::now();
          auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
          if (duration > WAIT_FOR_PROGRAM_RUNNING_TIMEOUT)
          {
            break;
          }
          // Wait for program to be running
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        if (!isProgramRunning())
        {
          disconnect();
          throw std::logic_error("RTDE control program is not running on controller, before timeout of " +
                                 std::to_string(WAIT_FOR_PROGRAM_RUNNING_TIMEOUT) + " seconds");
        }
      }
    }
  }
#else
  if (!upload_script_ && use_external_control_ur_cap_)
  {
    throw std::logic_error("The use of ExternalControl UR Cap is not supported on Windows and Apple yet. "
        "Please contact author");
  }
#endif

  // When the user wants to a custom script / program on the controller interacting with ur_rtde.
  if (!upload_script_ && !use_external_control_ur_cap_)
  {
    if (!no_wait_)
    {
      if (!isProgramRunning())
      {
        start_time = std::chrono::high_resolution_clock::now();
        std::cout << "Waiting for RTDE control program to be running on the controller" << std::endl;
        while (!isProgramRunning())
        {
          std::chrono::high_resolution_clock::time_point current_time = std::chrono::high_resolution_clock::now();
          auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
          if (duration > WAIT_FOR_PROGRAM_RUNNING_TIMEOUT)
          {
            break;
          }
          // Wait for program to be running
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        if (!isProgramRunning())
        {
          disconnect();
          throw std::logic_error("RTDE control program is not running on controller, before timeout of " +
                                 std::to_string(WAIT_FOR_PROGRAM_RUNNING_TIMEOUT) + " seconds");
        }
      }
    }
  }

  return true;
}

bool RTDEControlInterface::setupRecipes(const double &frequency)
{
  // Setup output
  state_names_ = {"robot_status_bits", "safety_status_bits", "runtime_state", outIntReg(0),
                                          outIntReg(1),        outDoubleReg(0),      outDoubleReg(1), outDoubleReg(2),
                                          outDoubleReg(3),     outDoubleReg(4),      outDoubleReg(5)};
  rtde_->sendOutputSetup(state_names_, frequency);

  // Setup input recipes
  // Recipe 1
  std::vector<std::string> async_setp_input = {inIntReg(0),    inDoubleReg(0), inDoubleReg(1), inDoubleReg(2),
                                               inDoubleReg(3), inDoubleReg(4), inDoubleReg(5), inDoubleReg(6),
                                               inDoubleReg(7), inIntReg(1)};
  rtde_->sendInputSetup(async_setp_input);

  // Recipe 2
  std::vector<std::string> servoj_input = {inIntReg(0),    inDoubleReg(0), inDoubleReg(1), inDoubleReg(2),
                                           inDoubleReg(3), inDoubleReg(4), inDoubleReg(5), inDoubleReg(6),
                                           inDoubleReg(7), inDoubleReg(8), inDoubleReg(9), inDoubleReg(10)};
  rtde_->sendInputSetup(servoj_input);

  // Recipe 3
  std::vector<std::string> force_mode_input = {
      inIntReg(0),     inIntReg(1),     inIntReg(2),     inIntReg(3),     inIntReg(4),     inIntReg(5),
      inIntReg(6),     inIntReg(7),     inDoubleReg(0),  inDoubleReg(1),  inDoubleReg(2),  inDoubleReg(3),
      inDoubleReg(4),  inDoubleReg(5),  inDoubleReg(6),  inDoubleReg(7),  inDoubleReg(8),  inDoubleReg(9),
      inDoubleReg(10), inDoubleReg(11), inDoubleReg(12), inDoubleReg(13), inDoubleReg(14), inDoubleReg(15),
      inDoubleReg(16), inDoubleReg(17)};
  rtde_->sendInputSetup(force_mode_input);

  // Recipe 4
  std::vector<std::string> no_cmd_input = {inIntReg(0)};
  rtde_->sendInputSetup(no_cmd_input);

  // Recipe 5
  std::vector<std::string> servoc_input = {inIntReg(0),    inDoubleReg(0), inDoubleReg(1), inDoubleReg(2),
                                           inDoubleReg(3), inDoubleReg(4), inDoubleReg(5), inDoubleReg(6),
                                           inDoubleReg(7), inDoubleReg(8)};
  rtde_->sendInputSetup(servoc_input);

  // Recipe 6
  std::vector<std::string> wrench_input = {inIntReg(0),    inDoubleReg(0), inDoubleReg(1), inDoubleReg(2),
                                           inDoubleReg(3), inDoubleReg(4), inDoubleReg(5)};
  rtde_->sendInputSetup(wrench_input);

  // Recipe 7
  std::vector<std::string> set_payload_input = {inIntReg(0), inDoubleReg(0), inDoubleReg(1), inDoubleReg(2),
                                                inDoubleReg(3)};
  rtde_->sendInputSetup(set_payload_input);

  // Recipe 8
  std::vector<std::string> force_mode_parameters_input = {inIntReg(0), inDoubleReg(0)};
  rtde_->sendInputSetup(force_mode_parameters_input);

  // Recipe 9
  std::vector<std::string> get_actual_joint_positions_history_input = {inIntReg(0), inIntReg(1)};
  rtde_->sendInputSetup(get_actual_joint_positions_history_input);

  // Recipe 10
  std::vector<std::string> get_inverse_kin_input = {inIntReg(0),     inDoubleReg(0),  inDoubleReg(1), inDoubleReg(2),
                                                    inDoubleReg(3),  inDoubleReg(4),  inDoubleReg(5), inDoubleReg(6),
                                                    inDoubleReg(7),  inDoubleReg(8),  inDoubleReg(9), inDoubleReg(10),
                                                    inDoubleReg(11), inDoubleReg(12), inDoubleReg(13)};
  rtde_->sendInputSetup(get_inverse_kin_input);

  // Recipe 11
  std::vector<std::string> watchdog_input = {inIntReg(0)};
  rtde_->sendInputSetup(watchdog_input);

  // Recipe 12
  std::vector<std::string> pose_trans_input = {
      inIntReg(0),    inDoubleReg(0), inDoubleReg(1), inDoubleReg(2), inDoubleReg(3),  inDoubleReg(4), inDoubleReg(5),
      inDoubleReg(6), inDoubleReg(7), inDoubleReg(8), inDoubleReg(9), inDoubleReg(10), inDoubleReg(11)};
  rtde_->sendInputSetup(pose_trans_input);

  // Recipe 13
  std::vector<std::string> setp_input = {inIntReg(0),    inDoubleReg(0), inDoubleReg(1), inDoubleReg(2), inDoubleReg(3),
                                         inDoubleReg(4), inDoubleReg(5), inDoubleReg(6), inDoubleReg(7)};
  rtde_->sendInputSetup(setp_input);

  // Recipe 14
  std::vector<std::string> jog_input = {inIntReg(0),    inDoubleReg(0), inDoubleReg(1), inDoubleReg(2),
                                        inDoubleReg(3), inDoubleReg(4), inDoubleReg(5), inDoubleReg(6),
                                        inDoubleReg(7),  inDoubleReg(8), inDoubleReg(9), inDoubleReg(10),
                                        inDoubleReg(11), inDoubleReg(12)};
  rtde_->sendInputSetup(jog_input);

  // Recipe 15
  std::vector<std::string> async_path_input = {inIntReg(0), inIntReg(1)};
  rtde_->sendInputSetup(async_path_input);

  // Recipe 16
  std::vector<std::string> move_until_contact_input = {inIntReg(0),     inDoubleReg(0), inDoubleReg(1), inDoubleReg(2),
                                                       inDoubleReg(3),  inDoubleReg(4), inDoubleReg(5), inDoubleReg(6),
                                                       inDoubleReg(7),  inDoubleReg(8), inDoubleReg(9), inDoubleReg(10),
                                                       inDoubleReg(11), inDoubleReg(12)};
  rtde_->sendInputSetup(move_until_contact_input);

  return true;
}

void RTDEControlInterface::receiveCallback()
{
  while (!stop_thread_)
  {
    // Receive and update the robot state
    try
    {
      rtde_->receiveData(robot_state_);
      // temporary hack to fix synchronization problems on windows.
#ifndef _WIN32
      std::this_thread::sleep_for(std::chrono::microseconds(100));
#endif
    }
    catch (std::exception &e)
    {
      std::cerr << "RTDEControlInterface: Could not receive data from robot..." << std::endl;
      std::cerr << e.what() << std::endl;
      if (rtde_ != nullptr)
      {
        if (rtde_->isConnected())
          rtde_->disconnect();

        if (!rtde_->isConnected())
        {
          std::cerr << "RTDEControlInterface: Robot is disconnected, reconnecting..." << std::endl;
          reconnect();
        }

        if (rtde_->isConnected())
          std::cout << "RTDEControlInterface: Successfully reconnected!" << std::endl;
        else
          throw std::runtime_error("Could not recover from losing connection to robot!");
      }
    }
  }
}

void RTDEControlInterface::stopScript()
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::STOP_SCRIPT;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_4;
  sendCommand(robot_cmd);
}

void RTDEControlInterface::stopL(double a)
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::STOPL;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_8;
  robot_cmd.val_.push_back(a);
  sendCommand(robot_cmd);
}

void RTDEControlInterface::stopJ(double a)
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::STOPJ;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_8;
  robot_cmd.val_.push_back(a);
  sendCommand(robot_cmd);
}

bool RTDEControlInterface::reuploadScript()
{
  if (isProgramRunning())
  {
    if (verbose_)
      std::cout << "A script was running on the controller, killing it!" << std::endl;

    // Stop the running script first
    stopScript();
    db_client_->stop();

    // Wait until terminated
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Re-upload RTDE script to the UR Controller
  if (script_client_->sendScript())
  {
    if (verbose_)
      std::cout << "The RTDE Control script has been re-uploaded." << std::endl;
    return true;
  }
  else
  {
    return false;
  }
}

bool RTDEControlInterface::sendCustomScriptFunction(const std::string &function_name, const std::string &script)
{
  std::string cmd_str;
  std::string line;
  std::stringstream ss(script);
  cmd_str += "def " + function_name + "():\n";
  cmd_str += "\twrite_output_integer_register(0 +" + std::to_string(register_offset_) + ", 1)\n";

  while (std::getline(ss, line))
  {
    cmd_str += "\t" + line + "\n";
  }

  // Signal when motions are finished
  cmd_str += "\twrite_output_integer_register(0 +" + std::to_string(register_offset_) + ", 2)\n";
  cmd_str += "end\n";

  return sendCustomScript(cmd_str);
}

bool RTDEControlInterface::sendCustomScript(const std::string &script)
{
  custom_script_running_ = true;
  // First stop the running RTDE control script
  stopScript();

  auto start_time = std::chrono::high_resolution_clock::now();

  // Send custom script function
  script_client_->sendScriptCommand(script);

  while (getControlScriptState() != UR_CONTROLLER_DONE_WITH_CMD)
  {
    // Wait until the controller is done with command
    auto current_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
    if (duration > UR_PATH_EXECUTION_TIMEOUT)
      return false;
    // Sleep to avoid high CPU load
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  sendClearCommand();

  // Re-upload RTDE script to the UR Controller
  script_client_->sendScript();

  while (!isProgramRunning())
  {
    // Wait for program to be running
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  custom_script_running_ = false;
  return true;
}

bool RTDEControlInterface::sendCustomScriptFile(const std::string &file_path)
{
  custom_script_running_ = true;
  // First stop the running RTDE control script
  stopScript();

  auto start_time = std::chrono::high_resolution_clock::now();

  // Send custom script file
  script_client_->sendScript(file_path);

  while (getControlScriptState() != UR_CONTROLLER_DONE_WITH_CMD)
  {
    // Wait until the controller is done with command
    auto current_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
    if (duration > UR_PATH_EXECUTION_TIMEOUT)
      return false;
    // Sleep to avoid high CPU load
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  sendClearCommand();

  // Re-upload RTDE script to the UR Controller
  script_client_->sendScript();

  while (!isProgramRunning())
  {
    // Wait for program to be running
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  custom_script_running_ = false;
  return true;
}

RTDE_EXPORT void RTDEControlInterface::setCustomScriptFile(const std::string &file_path)
{
  script_client_->setScriptFile(file_path);
  reuploadScript();
}

void RTDEControlInterface::verifyValueIsWithin(const double &value, const double &min, const double &max)
{
  if (std::isnan(min) || std::isnan(max))
  {
    throw std::invalid_argument("Make sure both min and max are not NaN's");
  }
  else if (std::isnan(value))
  {
    throw std::invalid_argument("The value is considered NaN");
  }
  else if (!(std::isgreaterequal(value, min) && std::islessequal(value, max)))
  {
    std::ostringstream oss;
    oss << "The value is not within [" << min << ";" << max << "]";
    throw std::range_error(oss.str());
  }
}

std::string RTDEControlInterface::buildPathScriptCode(const std::vector<std::vector<double>> &path,
                                                      const std::string &cmd)
{
  std::stringstream ss;
  for (const auto &pose : path)
  {
    if (cmd == "movej(")
    {
      verifyValueIsWithin(pose[6], UR_JOINT_VELOCITY_MIN, UR_JOINT_VELOCITY_MAX);
      verifyValueIsWithin(pose[7], UR_JOINT_ACCELERATION_MIN, UR_JOINT_ACCELERATION_MAX);
      verifyValueIsWithin(pose[8], UR_BLEND_MIN, UR_BLEND_MAX);
    }
    else if (cmd == "movel(p")
    {
      verifyValueIsWithin(pose[6], UR_TOOL_VELOCITY_MIN, UR_TOOL_VELOCITY_MAX);
      verifyValueIsWithin(pose[7], UR_TOOL_ACCELERATION_MIN, UR_TOOL_ACCELERATION_MAX);
      verifyValueIsWithin(pose[8], UR_BLEND_MIN, UR_BLEND_MAX);
    }
    ss << "\t" << cmd << "[" << pose[0] << "," << pose[1] << "," << pose[2] << "," << pose[3] << "," << pose[4] << ","
       << pose[5] << "],"
       << "a=" << pose[7] << ",v=" << pose[6] << ",r=" << pose[8] << ")\n";
  }
  return ss.str();
}

bool RTDEControlInterface::moveJ(const std::vector<std::vector<double>> &path, bool async)
{
  Path NewPath;
  NewPath.appendMovejPath(path);
  auto PathScript = NewPath.toScriptCode();
  if (verbose_)
    std::cout << "PathScript: ----------------------------------------------\n" << PathScript << "\n\n" << std::endl;

  custom_script_running_ = true;
  // stop the running RTDE control script
  stopScript();
  // now inject the movej path into the main UR script
  script_client_->setScriptInjection(move_path_inject_id, PathScript);
  // Re-upload RTDE script to the UR Controller
  script_client_->sendScript();
  while (!isProgramRunning())
  {
    // Wait for program to be running
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  custom_script_running_ = false;

  // Now send the command
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::MOVE_PATH;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_15;
  robot_cmd.async_ = async ? 1 : 0;
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::movePath(const Path &path, bool async)
{
  // This is the first step because it may throw an exception
  auto path_script = path.toScriptCode();
  if (verbose_)
    std::cout << "path_script: ----------------------------------------------\n" << path_script << "\n\n" << std::endl;

  custom_script_running_ = true;
  // stop the running RTDE control script
  stopScript();
  // now inject the movej path into the main UR script
  script_client_->setScriptInjection(move_path_inject_id, path_script);
  // Re-upload RTDE script to the UR Controller
  script_client_->sendScript();
  while (!isProgramRunning())
  {
    // Wait for program to be running
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  custom_script_running_ = false;

  // Now send the command
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::MOVE_PATH;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_15;
  robot_cmd.async_ = async ? 1 : 0;
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::moveJ(const std::vector<double> &q, double speed, double acceleration, bool async)
{
  verifyValueIsWithin(speed, UR_JOINT_VELOCITY_MIN, UR_JOINT_VELOCITY_MAX);
  verifyValueIsWithin(acceleration, UR_JOINT_ACCELERATION_MIN, UR_JOINT_ACCELERATION_MAX);

  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::MOVEJ;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_1;
  if (async)
    robot_cmd.async_ = 1;
  else
    robot_cmd.async_ = 0;
  robot_cmd.val_ = q;
  robot_cmd.val_.push_back(speed);
  robot_cmd.val_.push_back(acceleration);
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::moveJ_IK(const std::vector<double> &transform, double speed, double acceleration, bool async)
{
  verifyValueIsWithin(speed, UR_JOINT_VELOCITY_MIN, UR_JOINT_VELOCITY_MAX);
  verifyValueIsWithin(acceleration, UR_JOINT_ACCELERATION_MIN, UR_JOINT_ACCELERATION_MAX);

  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::MOVEJ_IK;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_1;
  if (async)
    robot_cmd.async_ = 1;
  else
    robot_cmd.async_ = 0;
  robot_cmd.val_ = transform;
  robot_cmd.val_.push_back(speed);
  robot_cmd.val_.push_back(acceleration);
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::moveL(const std::vector<std::vector<double>> &path, bool async)
{
  Path NewPath;
  NewPath.appendMovelPath(path);
  auto PathScript = NewPath.toScriptCode();
  if (verbose_)
    std::cout << "Path: ----------------------------------------------\n" << PathScript << "\n\n" << std::endl;

  custom_script_running_ = true;
  // stop the running RTDE control script
  stopScript();
  // now inject the movel path into the main UR script
  script_client_->setScriptInjection(move_path_inject_id, PathScript);
  // Re-upload RTDE script to the UR Controller
  script_client_->sendScript();
  while (!isProgramRunning())
  {
    // Wait for program to be running
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  custom_script_running_ = false;

  // Now send the command
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::MOVE_PATH;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_15;
  robot_cmd.async_ = async ? 1 : 0;
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::moveL(const std::vector<double> &transform, double speed, double acceleration, bool async)
{
  verifyValueIsWithin(speed, UR_TOOL_VELOCITY_MIN, UR_TOOL_VELOCITY_MAX);
  verifyValueIsWithin(acceleration, UR_TOOL_ACCELERATION_MIN, UR_TOOL_ACCELERATION_MAX);

  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::MOVEL;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_1;
  if (async)
    robot_cmd.async_ = 1;
  else
    robot_cmd.async_ = 0;
  robot_cmd.val_ = transform;
  robot_cmd.val_.push_back(speed);
  robot_cmd.val_.push_back(acceleration);
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::jogStart(const std::vector<double> &speeds, int feature,
	const std::vector<double>& custom_frame)
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::JOG_START;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_14;
  robot_cmd.val_ = speeds;
  robot_cmd.val_.push_back(feature);
  if (!custom_frame.empty())
  {
    for (const auto &val : custom_frame)
      robot_cmd.val_.push_back(val);
  }
  else
  {
	for (int i = 0; i < 6; ++i)
      robot_cmd.val_.push_back(0);
  }
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::jogStop()
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::JOG_STOP;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_4;
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::moveL_FK(const std::vector<double> &q, double speed, double acceleration, bool async)
{
  verifyValueIsWithin(speed, UR_TOOL_VELOCITY_MIN, UR_TOOL_VELOCITY_MAX);
  verifyValueIsWithin(acceleration, UR_TOOL_ACCELERATION_MIN, UR_TOOL_ACCELERATION_MAX);

  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::MOVEL_FK;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_1;
  if (async)
    robot_cmd.async_ = 1;
  else
    robot_cmd.async_ = 0;
  robot_cmd.val_ = q;
  robot_cmd.val_.push_back(speed);
  robot_cmd.val_.push_back(acceleration);
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::forceMode(const std::vector<double> &task_frame, const std::vector<int> &selection_vector,
                                     const std::vector<double> &wrench, int type, const std::vector<double> &limits)
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::FORCE_MODE;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_3;
  robot_cmd.val_ = task_frame;
  for (const auto &val : wrench)
    robot_cmd.val_.push_back(val);

  for (const auto &val : limits)
    robot_cmd.val_.push_back(val);

  robot_cmd.selection_vector_ = selection_vector;
  robot_cmd.force_mode_type_ = type;
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::forceModeStop()
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::FORCE_MODE_STOP;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_4;
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::zeroFtSensor()
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::ZERO_FT_SENSOR;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_4;
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::speedJ(const std::vector<double> &qd, double acceleration, double time)
{
  verifyValueIsWithin(acceleration, UR_JOINT_ACCELERATION_MIN, UR_JOINT_ACCELERATION_MAX);

  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::SPEEDJ;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_13;
  robot_cmd.val_ = qd;
  robot_cmd.val_.push_back(acceleration);
  robot_cmd.val_.push_back(time);
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::speedL(const std::vector<double> &xd, double acceleration, double time)
{
  verifyValueIsWithin(acceleration, UR_TOOL_ACCELERATION_MIN, UR_TOOL_ACCELERATION_MAX);

  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::SPEEDL;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_13;
  robot_cmd.val_ = xd;
  robot_cmd.val_.push_back(acceleration);
  robot_cmd.val_.push_back(time);
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::servoJ(const std::vector<double> &q, double speed, double acceleration, double time,
                                  double lookahead_time, double gain)
{
  verifyValueIsWithin(speed, UR_JOINT_VELOCITY_MIN, UR_JOINT_VELOCITY_MAX);
  verifyValueIsWithin(acceleration, UR_JOINT_ACCELERATION_MIN, UR_JOINT_ACCELERATION_MAX);
  verifyValueIsWithin(lookahead_time, UR_SERVO_LOOKAHEAD_TIME_MIN, UR_SERVO_LOOKAHEAD_TIME_MAX);
  verifyValueIsWithin(gain, UR_SERVO_GAIN_MIN, UR_SERVO_GAIN_MAX);

  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::SERVOJ;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_2;
  robot_cmd.val_ = q;
  robot_cmd.val_.push_back(speed);
  robot_cmd.val_.push_back(acceleration);
  robot_cmd.val_.push_back(time);
  robot_cmd.val_.push_back(lookahead_time);
  robot_cmd.val_.push_back(gain);
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::servoL(const std::vector<double> &pose, double speed, double acceleration, double time,
                                  double lookahead_time, double gain)
{
  verifyValueIsWithin(speed, UR_JOINT_VELOCITY_MIN, UR_JOINT_VELOCITY_MAX);
  verifyValueIsWithin(acceleration, UR_JOINT_ACCELERATION_MIN, UR_JOINT_ACCELERATION_MAX);
  verifyValueIsWithin(lookahead_time, UR_SERVO_LOOKAHEAD_TIME_MIN, UR_SERVO_LOOKAHEAD_TIME_MAX);
  verifyValueIsWithin(gain, UR_SERVO_GAIN_MIN, UR_SERVO_GAIN_MAX);

  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::SERVOL;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_2;
  robot_cmd.val_ = pose;
  robot_cmd.val_.push_back(speed);
  robot_cmd.val_.push_back(acceleration);
  robot_cmd.val_.push_back(time);
  robot_cmd.val_.push_back(lookahead_time);
  robot_cmd.val_.push_back(gain);
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::speedStop(double a)
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::SPEED_STOP;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_8;
  robot_cmd.val_.push_back(a);
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::servoStop(double a)
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::SERVO_STOP;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_8;
  robot_cmd.val_.push_back(a);
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::servoC(const std::vector<double> &pose, double speed, double acceleration, double blend)
{
  verifyValueIsWithin(speed, UR_TOOL_VELOCITY_MIN, UR_TOOL_VELOCITY_MAX);
  verifyValueIsWithin(acceleration, UR_TOOL_ACCELERATION_MIN, UR_TOOL_ACCELERATION_MAX);
  verifyValueIsWithin(blend, UR_BLEND_MIN, UR_BLEND_MAX);

  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::SERVOC;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_5;
  robot_cmd.val_ = pose;
  robot_cmd.val_.push_back(speed);
  robot_cmd.val_.push_back(acceleration);
  robot_cmd.val_.push_back(blend);
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::setPayload(double mass, const std::vector<double> &cog)
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::SET_PAYLOAD;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_7;
  robot_cmd.val_.push_back(mass);
  if (!cog.empty())
  {
    for (const auto &val : cog)
      robot_cmd.val_.push_back(val);
  }
  else
  {
    robot_cmd.val_.push_back(0);
    robot_cmd.val_.push_back(0);
    robot_cmd.val_.push_back(0);
  }
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::teachMode()
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::TEACH_MODE;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_4;
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::endTeachMode()
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::END_TEACH_MODE;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_4;
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::forceModeSetDamping(double damping)
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::FORCE_MODE_SET_DAMPING;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_8;
  robot_cmd.val_.push_back(damping);
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::forceModeSetGainScaling(double scaling)
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::FORCE_MODE_SET_GAIN_SCALING;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_8;
  robot_cmd.val_.push_back(scaling);
  return sendCommand(robot_cmd);
}

int RTDEControlInterface::toolContact(const std::vector<double> &direction)
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::TOOL_CONTACT;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_6;
  robot_cmd.val_ = direction;
  if (sendCommand(robot_cmd))
  {
    return getToolContactValue();
  }
  else
  {
    return 0;
  }
}

double RTDEControlInterface::getStepTime()
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::GET_STEPTIME;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_4;
  if (sendCommand(robot_cmd))
  {
    return getStepTimeValue();
  }
  else
  {
    return 0;
  }
}

std::vector<double> RTDEControlInterface::getActualJointPositionsHistory(int steps)
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::GET_ACTUAL_JOINT_POSITIONS_HISTORY;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_9;
  robot_cmd.steps_ = steps;
  if (sendCommand(robot_cmd))
  {
    return getActualJointPositionsHistoryValue();
  }
  else
  {
    return std::vector<double>();
  }
}

std::vector<double> RTDEControlInterface::getTargetWaypoint()
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::GET_TARGET_WAYPOINT;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_4;
  if (sendCommand(robot_cmd))
  {
    return getTargetWaypointValue();
  }
  else
  {
    return std::vector<double>();
  }
}

bool RTDEControlInterface::isProgramRunning()
{
  if (robot_state_ != nullptr)
  {
    uint32_t robot_status;
    if (robot_state_->getStateData("robot_status_bits", robot_status))
    {
      // Read Bits 0-3: Is power on(1) | Is program running(2) | Is teach button pressed(4) | Is power button pressed(8)
      std::bitset<32> status_bits(robot_status);
      return status_bits.test(RobotStatus::ROBOT_STATUS_PROGRAM_RUNNING);
    }
    else
      throw std::runtime_error("unable to get state data for specified key: robot_status");
  }
  else
  {
    throw std::logic_error("Please initialize the RobotState, before using it!");
  }
}

double RTDEControlInterface::getStepTimeValue()
{
  if (robot_state_ != nullptr)
  {
    return getOutputDoubleReg(0);
  }
  else
  {
    throw std::logic_error("Please initialize the RobotState, before using it!");
  }
}

int RTDEControlInterface::getToolContactValue()
{
  if (robot_state_ != nullptr)
  {
    return getOutputIntReg(1);
  }
  else
  {
    throw std::logic_error("Please initialize the RobotState, before using it!");
  }
}

std::vector<double> RTDEControlInterface::getTargetWaypointValue()
{
  if (robot_state_ != nullptr)
  {
    std::vector<double> target_waypoint = {getOutputDoubleReg(0), getOutputDoubleReg(1), getOutputDoubleReg(2),
                                           getOutputDoubleReg(3), getOutputDoubleReg(4), getOutputDoubleReg(5)};
    return target_waypoint;
  }
  else
  {
    throw std::logic_error("Please initialize the RobotState, before using it!");
  }
}

std::vector<double> RTDEControlInterface::getActualJointPositionsHistoryValue()
{
  if (robot_state_ != nullptr)
  {
    std::vector<double> actual_joint_positions_history = {getOutputDoubleReg(0), getOutputDoubleReg(1),
                                                          getOutputDoubleReg(2), getOutputDoubleReg(3),
                                                          getOutputDoubleReg(4), getOutputDoubleReg(5)};
    return actual_joint_positions_history;
  }
  else
  {
    throw std::logic_error("Please initialize the RobotState, before using it!");
  }
}

std::vector<double> RTDEControlInterface::getInverseKinematicsValue()
{
  if (robot_state_ != nullptr)
  {
    std::vector<double> q = {getOutputDoubleReg(0), getOutputDoubleReg(1), getOutputDoubleReg(2),
                             getOutputDoubleReg(3), getOutputDoubleReg(4), getOutputDoubleReg(5)};
    return q;
  }
  else
  {
    throw std::logic_error("Please initialize the RobotState, before using it!");
  }
}

std::vector<double> RTDEControlInterface::poseTransValue()
{
  if (robot_state_ != nullptr)
  {
    std::vector<double> pose = {getOutputDoubleReg(0), getOutputDoubleReg(1), getOutputDoubleReg(2),
                                getOutputDoubleReg(3), getOutputDoubleReg(4), getOutputDoubleReg(5)};
    return pose;
  }
  else
  {
    throw std::logic_error("Please initialize the RobotState, before using it!");
  }
}

bool RTDEControlInterface::setTcp(const std::vector<double> &tcp_offset)
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::SET_TCP;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_6;
  robot_cmd.val_ = tcp_offset;
  return sendCommand(robot_cmd);
}

std::vector<double> RTDEControlInterface::getInverseKinematics(const std::vector<double> &x,
                                                               const std::vector<double> &qnear,
                                                               double max_position_error, double max_orientation_error)
{
  RTDE::RobotCommand robot_cmd;
  if (!qnear.empty())
  {
    robot_cmd.type_ = RTDE::RobotCommand::Type::GET_INVERSE_KINEMATICS_ARGS;
    robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_10;
    robot_cmd.val_ = x;
    robot_cmd.val_.insert(robot_cmd.val_.end(), qnear.begin(), qnear.end());
    robot_cmd.val_.push_back(max_position_error);
    robot_cmd.val_.push_back(max_orientation_error);
  }
  else
  {
    robot_cmd.type_ = RTDE::RobotCommand::Type::GET_INVERSE_KINEMATICS_DEFAULT;
    robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_6;
    robot_cmd.val_ = x;
  }

  if (sendCommand(robot_cmd))
  {
    return getInverseKinematicsValue();
  }
  else
  {
    return std::vector<double>();
  }
}

std::vector<double> RTDEControlInterface::poseTrans(const std::vector<double> &p_from,
                                                    const std::vector<double> &p_from_to)
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::POSE_TRANS;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_12;
  robot_cmd.val_ = p_from;
  robot_cmd.val_.insert(robot_cmd.val_.end(), p_from_to.begin(), p_from_to.end());
  if (sendCommand(robot_cmd))
  {
    return poseTransValue();
  }
  else
  {
    return std::vector<double>();
  }
}

int RTDEControlInterface::getControlScriptState()
{
  if (robot_state_ != nullptr)
  {
    return getOutputIntReg(0);
  }
  else
  {
    throw std::logic_error("Please initialize the RobotState, before using it!");
  }
}

bool RTDEControlInterface::isProtectiveStopped()
{
  if (robot_state_ != nullptr)
  {
    uint32_t safety_status_bits;
    if (robot_state_->getStateData("safety_status_bits", safety_status_bits))
    {
      std::bitset<32> safety_status_bitset(safety_status_bits);
      return safety_status_bitset.test(SafetyStatus::IS_PROTECTIVE_STOPPED);
    }
    else
      throw std::runtime_error("unable to get state data for specified key: safety_status_bits");
  }
  else
  {
    throw std::logic_error("Please initialize the RobotState, before using it!");
  }
}

bool RTDEControlInterface::isEmergencyStopped()
{
  if (robot_state_ != nullptr)
  {
    uint32_t safety_status_bits;
    if (robot_state_->getStateData("safety_status_bits", safety_status_bits))
    {
      std::bitset<32> safety_status_bitset(safety_status_bits);
      return safety_status_bitset.test(SafetyStatus::IS_EMERGENCY_STOPPED);
    }
    else
      throw std::runtime_error("unable to get state data for specified key: safety_status_bits");
  }
  else
  {
    throw std::logic_error("Please initialize the RobotState, before using it!");
  }
}

bool RTDEControlInterface::triggerProtectiveStop()
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::PROTECTIVE_STOP;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_4;
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::setWatchdog(double min_frequency)
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::SET_WATCHDOG;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_8;
  robot_cmd.val_.push_back(min_frequency);
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::kickWatchdog()
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::WATCHDOG;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_11;
  return sendCommand(robot_cmd);
}

bool RTDEControlInterface::isPoseWithinSafetyLimits(const std::vector<double> &pose)
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::IS_POSE_WITHIN_SAFETY_LIMITS;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_6;
  robot_cmd.val_ = pose;

  if (sendCommand(robot_cmd))
  {
    if (robot_state_ != nullptr)
    {
      return getOutputIntReg(1) == 1;
    }
    else
    {
      throw std::logic_error("Please initialize the RobotState, before using it!");
    }
  }
  else
  {
    return false;
  }
}

bool RTDEControlInterface::isJointsWithinSafetyLimits(const std::vector<double> &q)
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::IS_JOINTS_WITHIN_SAFETY_LIMITS;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_6;
  robot_cmd.val_ = q;

  if (sendCommand(robot_cmd))
  {
    if (robot_state_ != nullptr)
    {
      return getOutputIntReg(1) == 1;
    }
    else
    {
      throw std::logic_error("Please initialize the RobotState, before using it!");
    }
  }
  else
  {
    return false;
  }
}

std::vector<double> RTDEControlInterface::getJointTorques()
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::GET_JOINT_TORQUES;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_4;

  if (sendCommand(robot_cmd))
  {
    if (robot_state_ != nullptr)
    {
      std::vector<double> torques = {getOutputDoubleReg(0), getOutputDoubleReg(1), getOutputDoubleReg(2),
                                     getOutputDoubleReg(3), getOutputDoubleReg(4), getOutputDoubleReg(5)};
      return torques;
    }
    else
    {
      throw std::logic_error("Please initialize the RobotState, before using it!");
    }
  }
  else
  {
    return std::vector<double>();
  }
}

std::vector<double> RTDEControlInterface::getTCPOffset()
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::GET_TCP_OFFSET;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_4;

  if (sendCommand(robot_cmd))
  {
    if (robot_state_ != nullptr)
    {
      std::vector<double> tcp_offset = {getOutputDoubleReg(0), getOutputDoubleReg(1), getOutputDoubleReg(2),
                                        getOutputDoubleReg(3), getOutputDoubleReg(4), getOutputDoubleReg(5)};
      return tcp_offset;
    }
    else
    {
      throw std::logic_error("Please initialize the RobotState, before using it!");
    }
  }
  else
  {
    return std::vector<double>();
  }
}

std::vector<double> RTDEControlInterface::getForwardKinematics(const std::vector<double> &q,
                                                               const std::vector<double> &tcp_offset)
{
  RTDE::RobotCommand robot_cmd;
  if (q.empty() && tcp_offset.empty())
  {
    robot_cmd.type_ = RTDE::RobotCommand::Type::GET_FORWARD_KINEMATICS_DEFAULT;
    robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_4;
  }
  else if (tcp_offset.empty() && !q.empty())
  {
    robot_cmd.type_ = RTDE::RobotCommand::Type::GET_FORWARD_KINEMATICS_ARGS;
    robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_6;
    robot_cmd.val_ = q;
  }
  else
  {
    robot_cmd.type_ = RTDE::RobotCommand::Type::GET_FORWARD_KINEMATICS_ARGS;
    robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_12;
    robot_cmd.val_ = q;
    robot_cmd.val_.insert(robot_cmd.val_.end(), tcp_offset.begin(), tcp_offset.end());
  }

  if (sendCommand(robot_cmd))
  {
    if (robot_state_ != nullptr)
    {
      std::vector<double> forward_kin = {getOutputDoubleReg(0), getOutputDoubleReg(1), getOutputDoubleReg(2),
                                         getOutputDoubleReg(3), getOutputDoubleReg(4), getOutputDoubleReg(5)};
      return forward_kin;
    }
    else
    {
      throw std::logic_error("Please initialize the RobotState, before using it!");
    }
  }
  else
  {
    return std::vector<double>();
  }
}

bool RTDEControlInterface::isSteady()
{
  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::IS_STEADY;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_4;

  if (sendCommand(robot_cmd))
  {
    if (robot_state_ != nullptr)
    {
      return getOutputIntReg(1) == 1;
    }
    else
    {
      throw std::logic_error("Please initialize the RobotState, before using it!");
    }
  }
  else
  {
    return false;
  }
}

bool RTDEControlInterface::moveUntilContact(const std::vector<double> &xd, const std::vector<double> &direction,
                                            double acceleration)
{
  verifyValueIsWithin(acceleration, UR_TOOL_ACCELERATION_MIN, UR_TOOL_ACCELERATION_MAX);

  RTDE::RobotCommand robot_cmd;
  robot_cmd.type_ = RTDE::RobotCommand::Type::MOVE_UNTIL_CONTACT;
  robot_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_16;
  robot_cmd.val_ = xd;
  for (const auto &val : direction)
    robot_cmd.val_.push_back(val);

  robot_cmd.val_.push_back(acceleration);
  return sendCommand(robot_cmd);
}

void RTDEControlInterface::unlockProtectiveStop() {
  db_client_->unlockProtectiveStop();
}

std::string RTDEControlInterface::outDoubleReg(int reg) const
{
  return "output_double_register_" + std::to_string(register_offset_ + reg);
};

std::string RTDEControlInterface::outIntReg(int reg) const
{
  return "output_int_register_" + std::to_string(register_offset_ + reg);
};

std::string RTDEControlInterface::inDoubleReg(int reg) const
{
  return "input_double_register_" + std::to_string(register_offset_ + reg);
};

std::string RTDEControlInterface::inIntReg(int reg) const
{
  return "input_int_register_" + std::to_string(register_offset_ + reg);
};

double RTDEControlInterface::getOutputDoubleReg(int reg)
{
  std::string output_double_register_key = "output_double_register_" + std::to_string(register_offset_ + reg);
  double output_double_register_val;
  if (robot_state_->getStateData(output_double_register_key, output_double_register_val))
    return output_double_register_val;
  else
    throw std::runtime_error("unable to get state data for specified key: "+output_double_register_key);
};

int RTDEControlInterface::getOutputIntReg(int reg)
{
  std::string output_int_register_key = "output_int_register_" + std::to_string(register_offset_ + reg);
  int32_t output_int_register_val;
  if (robot_state_->getStateData(output_int_register_key, output_int_register_val))
    return output_int_register_val;
  else
    throw std::runtime_error("unable to get state data for specified key: "+output_int_register_key);
};

bool RTDEControlInterface::sendCommand(const RTDE::RobotCommand &cmd)
{
  std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();

  try
  {
    uint32_t runtime_state;
    if (!robot_state_->getStateData("runtime_state", runtime_state))
      throw std::runtime_error("unable to get state data for specified key: runtime_state");

    if (runtime_state == RuntimeState::STOPPED)
    {
      if (!custom_script_running_)
      {
        sendClearCommand();
        return false;
      }
    }

    if (isProgramRunning() || custom_script_ || custom_script_running_ || use_external_control_ur_cap_)
    {
      while (getControlScriptState() != UR_CONTROLLER_RDY_FOR_CMD)
      {
        // If robot is in an emergency or protective stop return false
        if (isProtectiveStopped() || isEmergencyStopped())
        {
          sendClearCommand();
          return false;
        }

        // Wait until the controller is ready for a command or timeout
        std::chrono::high_resolution_clock::time_point current_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
        if (duration > UR_GET_READY_TIMEOUT)
        {
          sendClearCommand();
          return false;
        }
      }

      if (cmd.type_ == RTDE::RobotCommand::Type::SERVOJ || cmd.type_ == RTDE::RobotCommand::Type::SERVOL ||
          cmd.type_ == RTDE::RobotCommand::Type::SERVOC || cmd.type_ == RTDE::RobotCommand::Type::SPEEDJ ||
          cmd.type_ == RTDE::RobotCommand::Type::SPEEDL || cmd.type_ == RTDE::RobotCommand::Type::FORCE_MODE ||
          cmd.type_ == RTDE::RobotCommand::Type::WATCHDOG || cmd.type_ == RTDE::RobotCommand::Type::GET_JOINT_TORQUES ||
        cmd.type_ == RTDE::RobotCommand::Type::TOOL_CONTACT || cmd.type_ == RTDE::RobotCommand::Type::GET_STEPTIME ||
          cmd.type_ == RTDE::RobotCommand::Type::GET_ACTUAL_JOINT_POSITIONS_HISTORY)
      {
        // Send command to the controller
        rtde_->send(cmd);

        // We do not wait for 'continuous' / RT commands to finish.

        return true;
      }
      else
      {
        // Send command to the controller
        rtde_->send(cmd);

        if (cmd.type_ != RTDE::RobotCommand::Type::STOP_SCRIPT)
        {
          start_time = std::chrono::high_resolution_clock::now();
          while (getControlScriptState() != UR_CONTROLLER_DONE_WITH_CMD)
          {
            // if the script causes an error, for example because of inverse
            // kinematics calculation failed, then it may be that the script no
            // longer runs an we will never receive the UR_CONTROLLER_DONE_WITH_CMD
            // signal
            if (!isProgramRunning())
            {
              std::cerr << "RTDEControlInterface: RTDE control script is not running!" << std::endl;
              sendClearCommand();
              return false;
            }

            // If robot is in an emergency or protective stop return false
            if (isProtectiveStopped() || isEmergencyStopped())
            {
              sendClearCommand();
              return false;
            }

            // Wait until the controller has finished executing or timeout
            auto current_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
            if (duration > UR_EXECUTION_TIMEOUT)
            {
              sendClearCommand();
              return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
        }
        else
        {
          if (use_external_control_ur_cap_)
          {
            // Program is allowed to still be running when using the ExternalControl UR Cap.
            // So we simply wait a bit for the stop script command to go through and clear the cmd register and return.
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            sendClearCommand();
            return true;
          }
          else
          {
            while (isProgramRunning())
            {
              // If robot is in an emergency or protective stop return false
              if (isProtectiveStopped() || isEmergencyStopped())
              {
                sendClearCommand();
                return false;
              }

              // Wait for program to stop running or timeout
              auto current_time = std::chrono::high_resolution_clock::now();
              auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
              if (duration > UR_EXECUTION_TIMEOUT)
              {
                sendClearCommand();
                return false;
              }
              std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
        }

        // Make controller ready for next command
        sendClearCommand();
        return true;
      }
    }
    else
    {
      std::cerr << "RTDEControlInterface: RTDE control script is not running!" << std::endl;
      sendClearCommand();
      return false;
    }
  }
  catch (std::exception &e)
  {
    std::cerr << "RTDEControlInterface: Lost connection to robot..." << std::endl;
    std::cerr << e.what() << std::endl;
    if (rtde_ != nullptr)
    {
      if (rtde_->isConnected())
        rtde_->disconnect();
    }
  }

  if (!rtde_->isConnected())
  {
    std::cerr << "RTDEControlInterface: Robot is disconnected, reconnecting..." << std::endl;
    reconnect();
    return sendCommand(cmd);
  }
  sendClearCommand();
  return false;
}

void RTDEControlInterface::sendClearCommand()
{
  RTDE::RobotCommand clear_cmd;
  clear_cmd.type_ = RTDE::RobotCommand::Type::NO_CMD;
  clear_cmd.recipe_id_ = RTDE::RobotCommand::Recipe::RECIPE_4;
  rtde_->send(clear_cmd);
}

struct VelocityAccLimits
{
  double velocity_min;
  double velocity_max;
  double acceleration_min;
  double acceleration_max;
};

std::string PathEntry::toScriptCode() const
{
  static const VelocityAccLimits joint_limits = {UR_JOINT_VELOCITY_MIN, UR_JOINT_VELOCITY_MAX,
                                                 UR_JOINT_ACCELERATION_MIN, UR_JOINT_ACCELERATION_MAX};
  static const VelocityAccLimits tool_limits = {UR_TOOL_VELOCITY_MIN, UR_TOOL_VELOCITY_MAX, UR_TOOL_ACCELERATION_MIN,
                                                UR_TOOL_ACCELERATION_MAX};

  const VelocityAccLimits &limits = (PositionJoints == pos_type_) ? joint_limits : tool_limits;
  switch (move_type_)
  {
    case MoveJ:
    case MoveL:
    case MoveP:
      verifyValueIsWithin(param_[6], limits.velocity_min, limits.velocity_max);
      verifyValueIsWithin(param_[7], limits.acceleration_min, limits.acceleration_max);
      verifyValueIsWithin(param_[8], UR_BLEND_MIN, UR_BLEND_MAX);
      break;
    case MoveC:
      throw std::runtime_error("MoveC in path not supported yet");
      break;
  }

  std::stringstream ss;
  ss << "\t";
  switch (move_type_)
  {
    case MoveJ:
      ss << "movej(";
      break;
    case MoveL:
      ss << "movel(";
      break;
    case MoveP:
      ss << "movep(";
      break;
    case MoveC:
      ss << "movec(";
      break;
  }

  if (PositionTcpPose == pos_type_)
  {
    ss << "p";
  }

  ss << "[" << param_[0] << "," << param_[1] << "," << param_[2] << "," << param_[3] << "," << param_[4] << ","
     << param_[5] << "],"
     << "a=" << param_[7] << ",v=" << param_[6] << ",r=" << param_[8] << ")\n";
  return ss.str();
}

std::string Path::toScriptCode() const
{
  std::stringstream ss;
  for (size_t i = 0; i < waypoints_.size(); ++i)
  {
    ss << "\tsignal_async_progress(" << i << ")\n";
    ss << waypoints_[i].toScriptCode();
  }

  return ss.str();
}

void Path::addEntry(const PathEntry &entry)
{
  waypoints_.push_back(entry);
}

void Path::clear()
{
  waypoints_.clear();
}

std::size_t Path::size() const
{
  return waypoints_.size();
}

const std::vector<PathEntry> &Path::waypoints() const
{
  return waypoints_;
}

void Path::appendMovelPath(const std::vector<std::vector<double>> &path)
{
  for (const auto &move_l : path)
  {
    waypoints_.push_back(PathEntry(PathEntry::MoveL, PathEntry::PositionTcpPose, move_l));
  }
}

void Path::appendMovejPath(const std::vector<std::vector<double>> &path)
{
  for (const auto &move_j : path)
  {
    waypoints_.push_back(PathEntry(PathEntry::MoveJ, PathEntry::PositionJoints, move_j));
  }
}
}  // namespace ur_rtde
