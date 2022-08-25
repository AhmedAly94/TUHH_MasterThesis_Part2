#pragma once
#ifndef RTDE_H
#define RTDE_H

#include <ur_rtde/rtde_export.h>

#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// forward declarations
namespace ur_rtde
{
class RobotState;
}

namespace ur_rtde
{
namespace details
{
// convenience alias for callback functions used within RTDE::recieveData()
using cb_fun = std::function<void(std::shared_ptr<RobotState>, std::vector<char> &, uint32_t &)>;
// convenience alias for the callback map of string ids and callback funtion objects
using cb_map = std::unordered_map<std::string, details::cb_fun>;
}  // namespace details

class RTDE
{
 public:
  RTDE_EXPORT explicit RTDE(const std::string hostname, int port = 30004, bool verbose = false);

  RTDE_EXPORT virtual ~RTDE();

  class RobotCommand
  {
   public:
    enum Type
    {
      NO_CMD = 0,
      MOVEJ = 1,
      MOVEJ_IK = 2,
      MOVEL = 3,
      MOVEL_FK = 4,
      FORCE_MODE = 6,
      FORCE_MODE_STOP = 7,
      ZERO_FT_SENSOR = 8,
      SPEEDJ = 9,
      SPEEDL = 10,
      SERVOJ = 11,
      SERVOC = 12,
      SET_STD_DIGITAL_OUT = 13,
      SET_TOOL_DIGITAL_OUT = 14,
      SPEED_STOP = 15,
      SERVO_STOP = 16,
      SET_PAYLOAD = 17,
      TEACH_MODE = 18,
      END_TEACH_MODE = 19,
      FORCE_MODE_SET_DAMPING = 20,
      FORCE_MODE_SET_GAIN_SCALING = 21,
      SET_SPEED_SLIDER = 22,
      SET_STD_ANALOG_OUT = 23,
      SERVOL = 24,
      TOOL_CONTACT = 25,
      GET_STEPTIME = 26,
      GET_ACTUAL_JOINT_POSITIONS_HISTORY = 27,
      GET_TARGET_WAYPOINT = 28,
      SET_TCP = 29,
      GET_INVERSE_KINEMATICS_ARGS = 30,
      PROTECTIVE_STOP = 31,
      STOPL = 33,
      STOPJ = 34,
      SET_WATCHDOG = 35,
      IS_POSE_WITHIN_SAFETY_LIMITS = 36,
      IS_JOINTS_WITHIN_SAFETY_LIMITS = 37,
      GET_JOINT_TORQUES = 38,
      POSE_TRANS = 39,
      GET_TCP_OFFSET = 40,
      JOG_START = 41,
      JOG_STOP = 42,
      GET_FORWARD_KINEMATICS_DEFAULT = 43,
      GET_FORWARD_KINEMATICS_ARGS = 44,
      MOVE_PATH = 45,
      GET_INVERSE_KINEMATICS_DEFAULT = 46,
      IS_STEADY = 47,
      SET_CONF_DIGITAL_OUT = 48,
      SET_INPUT_INT_REGISTER = 49,
      SET_INPUT_DOUBLE_REGISTER = 50,
      MOVE_UNTIL_CONTACT = 51,
      WATCHDOG = 99,
      STOP_SCRIPT = 255
    };

    enum Recipe
    {
      RECIPE_1 = 1,
      RECIPE_2 = 2,
      RECIPE_3 = 3,
      RECIPE_4 = 4,
      RECIPE_5 = 5,
      RECIPE_6 = 6,
      RECIPE_7 = 7,
      RECIPE_8 = 8,
      RECIPE_9 = 9,
      RECIPE_10 = 10,
      RECIPE_11 = 11,
      RECIPE_12 = 12,
      RECIPE_13 = 13,
      RECIPE_14 = 14,
      RECIPE_15 = 15,
      RECIPE_16 = 16
    };

    RobotCommand() : type_(NO_CMD), recipe_id_(1)
    {
    }

    Type type_ = NO_CMD;
    std::uint8_t recipe_id_;
    std::int32_t async_;
    std::int32_t reg_int_val_;
    double reg_double_val_;
    std::vector<double> val_;
    std::vector<int> selection_vector_;
    std::int32_t force_mode_type_;
    std::uint8_t std_digital_out_;
    std::uint8_t std_digital_out_mask_;
    std::uint8_t configurable_digital_out_;
    std::uint8_t configurable_digital_out_mask_;
    std::uint8_t std_tool_out_;
    std::uint8_t std_tool_out_mask_;
    std::uint8_t std_analog_output_mask_;
    std::uint8_t std_analog_output_type_;
    double std_analog_output_0_;
    double std_analog_output_1_;
    std::int32_t speed_slider_mask_;
    double speed_slider_fraction_;
    std::uint32_t steps_;
  };

  enum RTDECommand
  {
    RTDE_REQUEST_PROTOCOL_VERSION = 86,       // ascii V
    RTDE_GET_URCONTROL_VERSION = 118,         // ascii v
    RTDE_TEXT_MESSAGE = 77,                   // ascii M
    RTDE_DATA_PACKAGE = 85,                   // ascii U
    RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS = 79,  // ascii O
    RTDE_CONTROL_PACKAGE_SETUP_INPUTS = 73,   // ascii I
    RTDE_CONTROL_PACKAGE_START = 83,          // ascii S
    RTDE_CONTROL_PACKAGE_PAUSE = 80           // ascii P
  };

  enum class ConnectionState : std::uint8_t
  {
    DISCONNECTED = 0,
    CONNECTED = 1,
    STARTED = 2,
    PAUSED = 3
  };

 public:
  RTDE_EXPORT void connect();
  RTDE_EXPORT void disconnect();
  RTDE_EXPORT bool isConnected();
  RTDE_EXPORT bool isStarted();

  RTDE_EXPORT bool negotiateProtocolVersion();
  RTDE_EXPORT std::tuple<std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t> getControllerVersion();
  RTDE_EXPORT void receive();
  RTDE_EXPORT boost::system::error_code receiveData(std::shared_ptr<RobotState> &robot_state);

  RTDE_EXPORT void send(const RobotCommand &robot_cmd);
  RTDE_EXPORT void sendAll(const std::uint8_t &command, std::string payload = "");
  RTDE_EXPORT void sendStart();
  RTDE_EXPORT void sendPause();
  RTDE_EXPORT bool sendOutputSetup(const std::vector<std::string> &output_names, double frequency);
  RTDE_EXPORT bool sendInputSetup(const std::vector<std::string> &input_names);

  std::string hostname_;
  int port_;
  bool verbose_;
  ConnectionState conn_state_;
  std::vector<std::string> output_types_;
  std::vector<std::string> output_names_;
  std::shared_ptr<boost::asio::io_service> io_service_;
  std::shared_ptr<boost::asio::ip::tcp::socket> socket_;
  std::shared_ptr<boost::asio::ip::tcp::resolver> resolver_;
  std::vector<char> buffer_;
};

}  // namespace ur_rtde

#endif  // RTDE_H
