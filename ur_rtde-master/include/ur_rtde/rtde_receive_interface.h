#pragma once
#ifndef RTDE_RECEIVE_INTERFACE_H
#define RTDE_RECEIVE_INTERFACE_H

#include <ur_rtde/rtde_export.h>
#include <ur_rtde/robot_state.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <functional>

#define MAJOR_VERSION 0
#define CB3_MAJOR_VERSION 3

// forward declarations
namespace boost
{
class thread;
}
namespace ur_rtde
{
class DashboardClient;
}
namespace ur_rtde
{
class RobotState;
}
namespace ur_rtde
{
class RTDE;
}

namespace ur_rtde
{
class RTDEReceiveInterface
{
 public:
  RTDE_EXPORT explicit RTDEReceiveInterface(std::string hostname, double frequency = -1.0,
                                            std::vector<std::string> variables = {},
                                            bool verbose = false, bool use_upper_range_registers = false);

  RTDE_EXPORT virtual ~RTDEReceiveInterface();

  enum SafetyStatus
  {
    IS_NORMAL_MODE = 0,
    IS_REDUCED_MODE = 1,
    IS_PROTECTIVE_STOPPED = 2,
    IS_RECOVERY_MODE = 3,
    IS_SAFEGUARD_STOPPED = 4,
    IS_SYSTEM_EMERGENCY_STOPPED = 5,
    IS_ROBOT_EMERGENCY_STOPPED = 6,
    IS_EMERGENCY_STOPPED = 7,
    IS_VIOLATION = 8,
    IS_FAULT = 9,
    IS_STOPPED_DUE_TO_SAFETY = 10
  };

  enum OutputIntRegisters
  {
    OUTPUT_INT_REGISTER_0 = 0,
    OUTPUT_INT_REGISTER_1,
    OUTPUT_INT_REGISTER_2,
    OUTPUT_INT_REGISTER_3,
    OUTPUT_INT_REGISTER_4,
    OUTPUT_INT_REGISTER_5,
    OUTPUT_INT_REGISTER_6,
    OUTPUT_INT_REGISTER_7,
    OUTPUT_INT_REGISTER_8,
    OUTPUT_INT_REGISTER_9,
    OUTPUT_INT_REGISTER_10,
    OUTPUT_INT_REGISTER_11,
    OUTPUT_INT_REGISTER_12,
    OUTPUT_INT_REGISTER_13,
    OUTPUT_INT_REGISTER_14,
    OUTPUT_INT_REGISTER_15,
    OUTPUT_INT_REGISTER_16,
    OUTPUT_INT_REGISTER_17,
    OUTPUT_INT_REGISTER_18,
    OUTPUT_INT_REGISTER_19,
    OUTPUT_INT_REGISTER_20,
    OUTPUT_INT_REGISTER_21,
    OUTPUT_INT_REGISTER_22,
    OUTPUT_INT_REGISTER_23
  };

  enum OutputDoubleRegisters
  {
    OUTPUT_DOUBLE_REGISTER_0 = 0,
    OUTPUT_DOUBLE_REGISTER_1,
    OUTPUT_DOUBLE_REGISTER_2,
    OUTPUT_DOUBLE_REGISTER_3,
    OUTPUT_DOUBLE_REGISTER_4,
    OUTPUT_DOUBLE_REGISTER_5,
    OUTPUT_DOUBLE_REGISTER_6,
    OUTPUT_DOUBLE_REGISTER_7,
    OUTPUT_DOUBLE_REGISTER_8,
    OUTPUT_DOUBLE_REGISTER_9,
    OUTPUT_DOUBLE_REGISTER_10,
    OUTPUT_DOUBLE_REGISTER_11,
    OUTPUT_DOUBLE_REGISTER_12,
    OUTPUT_DOUBLE_REGISTER_13,
    OUTPUT_DOUBLE_REGISTER_14,
    OUTPUT_DOUBLE_REGISTER_15,
    OUTPUT_DOUBLE_REGISTER_16,
    OUTPUT_DOUBLE_REGISTER_17,
    OUTPUT_DOUBLE_REGISTER_18,
    OUTPUT_DOUBLE_REGISTER_19,
    OUTPUT_DOUBLE_REGISTER_20,
    OUTPUT_DOUBLE_REGISTER_21,
    OUTPUT_DOUBLE_REGISTER_22,
    OUTPUT_DOUBLE_REGISTER_23
  };

  enum RuntimeState
  {
    STOPPING = 0,
    STOPPED = 1,
    PLAYING = 2,
    PAUSING = 3,
    PAUSED = 4,
    RESUMING = 5
  };

  enum class PausingState
  {
    PAUSED,
    RUNNING,
    RAMPUP
  };

  /**
   * @returns Can be used to disconnect from the robot. To reconnect you have to call the reconnect() function.
   */
  RTDE_EXPORT void disconnect();

  /**
   * @returns Can be used to reconnect to the robot after a lost connection.
   */
  RTDE_EXPORT bool reconnect();

  /**
   * @returns Can be used to reconnect to the robot after a lost connection.
   */
  RTDE_EXPORT bool startFileRecording(const std::string &filename, const std::vector<std::string> &variables={});

  /**
   * @returns Can be used to reconnect to the robot after a lost connection.
   */
  RTDE_EXPORT bool stopFileRecording();

  /**
   * @returns Connection status for RTDE, useful for checking for lost connection.
   */
  RTDE_EXPORT bool isConnected();

  /**
   * @returns Time elapsed since the controller was started [s]
   */
  RTDE_EXPORT double getTimestamp();

  /**
   * @returns Target joint positions
   */
  RTDE_EXPORT std::vector<double> getTargetQ();

  /**
   * @returns Target joint velocities
   */
  RTDE_EXPORT std::vector<double> getTargetQd();

  /**
   * @returns Target joint accelerations
   */
  RTDE_EXPORT std::vector<double> getTargetQdd();

  /**
   * @returns Target joint currents
   */
  RTDE_EXPORT std::vector<double> getTargetCurrent();

  /**
   * @returns Target joint moments (torques)
   */
  RTDE_EXPORT std::vector<double> getTargetMoment();

  /**
   * @returns Actual joint positions
   */
  RTDE_EXPORT std::vector<double> getActualQ();

  /**
   * @returns Actual joint velocities
   */
  RTDE_EXPORT std::vector<double> getActualQd();

  /**
   * @returns Actual joint currents
   */
  RTDE_EXPORT std::vector<double> getActualCurrent();

  /**
   * @returns Joint control currents
   */
  RTDE_EXPORT std::vector<double> getJointControlOutput();

  /**
   * @returns Actual Cartesian coordinates of the tool: (x,y,z,rx,ry,rz), where rx, ry and rz is a rotation vector
   * representation of the tool orientation
   */
  RTDE_EXPORT std::vector<double> getActualTCPPose();

  /**
   * @returns Actual speed of the tool given in Cartesian coordinates
   */
  RTDE_EXPORT std::vector<double> getActualTCPSpeed();

  /**
   * @returns Generalized forces in the TCP
   */
  RTDE_EXPORT std::vector<double> getActualTCPForce();

  /**
   * @returns Target Cartesian coordinates of the tool: (x,y,z,rx,ry,rz), where rx, ry and rz is a rotation vector
   * representation of the tool orientation
   */
  RTDE_EXPORT std::vector<double> getTargetTCPPose();

  /**
   * @returns Target speed of the tool given in Cartesian coordinates
   */
  RTDE_EXPORT std::vector<double> getTargetTCPSpeed();

  /**
   * @returns Current state of the digital inputs. 0-7: Standard, 8-15: Configurable, 16-17: Tool
   */
  RTDE_EXPORT uint64_t getActualDigitalInputBits();

  /**
   * @returns Temperature of each joint in degrees Celsius
   */
  RTDE_EXPORT std::vector<double> getJointTemperatures();

  /**
   * @returns Controller real-time thread execution time
   */
  RTDE_EXPORT double getActualExecutionTime();

  /**
   * @returns Robot mode
   * -1 = ROBOT_MODE_NO_CONTROLLER
   * 0 = ROBOT_MODE_DISCONNECTED
   * 1 = ROBOT_MODE_CONFIRM_SAFETY
   * 2	= ROBOT_MODE_BOOTING
   * 3 = ROBOT_MODE_POWER_OFF
   * 4 = ROBOT_MODE_POWER_ON
   * 5	= ROBOT_MODE_IDLE
   * 6	= ROBOT_MODE_BACKDRIVE
   * 7	= ROBOT_MODE_RUNNING
   * 8	= ROBOT_MODE_UPDATING_FIRMWARE
   */
  RTDE_EXPORT int32_t getRobotMode();

  /**
   * @returns Robot status
   * Bits 0-3: Is power on | Is program running | Is teach button pressed | Is power button pressed
   */
  RTDE_EXPORT uint32_t getRobotStatus();

  /**
   * @returns Joint control modes
   */
  RTDE_EXPORT std::vector<int32_t> getJointMode();

  /**
   * @returns Safety mode
   */
  RTDE_EXPORT int32_t getSafetyMode();

  /**
   * @returns Safety status bits
   * Bits 0-10: Is normal mode | Is reduced mode | Is protective stopped | Is recovery mode |
   * Is safeguard stopped | Is system emergency stopped | Is robot emergency stopped |
   * Is emergency stopped | Is violation | Is fault | Is stopped due to safety
   */
  RTDE_EXPORT uint32_t getSafetyStatusBits();

  /**
   * @returns Tool x, y and z accelerometer values
   */
  RTDE_EXPORT std::vector<double> getActualToolAccelerometer();

  /**
   * @returns Speed scaling of the trajectory limiter
   */
  RTDE_EXPORT double getSpeedScaling();

  /**
   * @returns Target speed fraction
   */
  RTDE_EXPORT double getTargetSpeedFraction();

  /**
   * @returns Norm of Cartesian linear momentum
   */
  RTDE_EXPORT double getActualMomentum();

  /**
   * @returns Safety Control Board: Main voltage
   */
  RTDE_EXPORT double getActualMainVoltage();

  /**
   * @returns Safety Control Board: Robot voltage (48V)
   */
  RTDE_EXPORT double getActualRobotVoltage();

  /**
   * @returns Safety Control Board: Robot current
   */
  RTDE_EXPORT double getActualRobotCurrent();

  /**
   * @returns Actual joint voltages
   */
  RTDE_EXPORT std::vector<double> getActualJointVoltage();

  /**
   * @returns Current state of the digital outputs. 0-7: Standard, 8-15: Configurable, 16-17: Tool
   */
  RTDE_EXPORT uint64_t getActualDigitalOutputBits();

  /** @brief Test if a digital output is set 'high' or 'low' the range is
   * 0-7: Standard, 8-15: Configurable, 16-17: Tool
   * @param output_id the id of the digital output to test
   * @returns a bool indicating the state of the digital output
   */
  RTDE_EXPORT bool getDigitalOutState(std::uint8_t output_id);

  /**
   * @returns Program state
   */
  RTDE_EXPORT uint32_t getRuntimeState();

  /**
   * @returns Standard analog input 0 [A or V]
   */
  RTDE_EXPORT double getStandardAnalogInput0();

  /**
   * @returns Standard analog input 1 [A or V]
   */
  RTDE_EXPORT double getStandardAnalogInput1();

  /**
   * @returns Standard analog output 0 [A or V]
   */
  RTDE_EXPORT double getStandardAnalogOutput0();

  /**
   * @returns Standard analog output 1 [A or V]
   */
  RTDE_EXPORT double getStandardAnalogOutput1();

  /**
   * @returns a bool indicating if the robot is in 'Protective stop'
   */
  RTDE_EXPORT bool isProtectiveStopped();

  /**
   * @returns a bool indicating if the robot is in 'Emergency stop'
   */
  RTDE_EXPORT bool isEmergencyStopped();

  /**
   * @brief Get the specified output integer register in either lower range [18-22] or upper range [42-46].
   * @param output_id the id of the register to read, current supported range is: [18-22] or [42-46], this can
   * be adjusted by changing the RTDEReceiveInterface output recipes and by using the use_upper_range_registers
   * constructor flag to switch between lower and upper range.
   * @returns an integer from the specified output register
   */
  RTDE_EXPORT int getOutputIntRegister(int output_id);

  /**
   * @brief Get the specified output double register in either lower range [18-22] or upper range [42-46].
   * @param output_id the id of the register to read, current supported range is: [18-22] or [42-46], this can
   * be adjusted by changing the RTDEReceiveInterface output recipes and by using the use_upper_range_registers
   * constructor flag to switch between lower and upper range.
   * @returns a double from the specified output register
   */
  RTDE_EXPORT double getOutputDoubleRegister(int output_id);

  /**
   * Reads progress information for asynchronous operations that supports
   * progress feedback (such as movePath).
   * @retval <0 Indicates that no async operation is running or
   * that an async operation has finished. The returned values of two 
   * consecutive async operations is never equal. Normally the 
   * returned values are toggled between -1 and -2. This allows the application
   * to clearly detect the end of an operation even if it is too short to see its
   * start. That means, if the value returned by this function is less than 0 
   * and is different from that last value returned by this function, then a
   * new async operation has finished.
   * @retval 0 Indicates that an async operation has started - progress 0
   * @retval >= 0 Indicates the progress of an async operation. For example,
   * if an operation has 3 steps, the progress ranges from 0 - 2. The progress
   * value is updated, before a step is executed. When the last step has been
   * executed, the value will change to -1 to indicate the end of the async
   * operation.
   */
  RTDE_EXPORT int getAsyncOperationProgress();

  /**
   * @brief Get the combined speed scaling
   * The combined speed scaling is the speed scaling resulting from multiplying the speed scaling
   * with the target speed fraction. The combined speed scaling takes the runtime_state of the
   * controller into account. If eg. a motion is paused on the teach pendant, and later
   * continued, the speed scaling will be ramped up from zero and return to
   * speed_scaling * target_speed_fraction when the runtime_state is RUNNING again.
   *
   * This is useful for scaling trajectories with the slider speed scaling currently set on the teach pendant.
   * @returns the actual combined speed scaling
   */
  RTDE_EXPORT double getSpeedScalingCombined();

  RTDE_EXPORT void receiveCallback();

  RTDE_EXPORT void recordCallback();

  const std::shared_ptr<RobotState>& robot_state() const {
    return robot_state_;
  }

 private:
  bool setupRecipes(const double& frequency);

  std::string outDoubleReg(int reg) const
  {
    return "output_double_register_" + std::to_string(register_offset_ + reg);
  };

  std::string outIntReg(int reg) const
  {
    return "output_int_register_" + std::to_string(register_offset_ + reg);
  };

  template <typename T>
  bool isWithinBounds(const T& value, const T& low, const T& high)
  {
    return (low <= value && value <= high);
  }

 private:
  std::string hostname_;
  double frequency_;
  std::vector<std::string> variables_;
  int port_;
  bool verbose_;
  bool use_upper_range_registers_;
  int register_offset_;
  double delta_time_;
  std::shared_ptr<RTDE> rtde_;
  std::atomic<bool> stop_receive_thread{false};
  std::atomic<bool> stop_record_thread{false};
  std::shared_ptr<boost::thread> th_;
  std::shared_ptr<boost::thread> record_thrd_;
  std::shared_ptr<RobotState> robot_state_;
  PausingState pausing_state_;
  std::shared_ptr<std::ofstream> file_recording_;
  std::vector<std::string> record_variables_;
  double speed_scaling_combined_{};
  double pausing_ramp_up_increment_;
};

}  // namespace ur_rtde

#endif  // RTDE_RECEIVE_INTERFACE_H
