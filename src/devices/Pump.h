#pragma once

#include <mc_rbdyn/Device.h>

#include <vector>

#include <chrono> //required for std::chrono::milliseconds

namespace mc_panda
{

/** This structure defines a pump vacuum gripper device equipped with a suction cup, 
 * that is a device that can vacuum and blowoff objects */
struct MC_RBDYN_DLLAPI Pump : public mc_rbdyn::Device
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /** Default constructor, does not represent a valid pump */
  inline Pump() : Pump("", "", sva::PTransformd::Identity()) {}

  /** Constructor
   *
   * @param name Name of the pump
   *
   * @param bodyName Name of the body to which the pump is attached
   *
   * @param X_b_s Transformation from the parent body to the pump
   *
   */
  inline Pump(const std::string & name, const std::string & bodyName, const sva::PTransformd & X_b_s)
  : mc_rbdyn::Device(name, bodyName, X_b_s)
  {
    type_ = "Pump";
  }

  ~Pump() override;

  /** Get the display's parent body name */
  inline const std::string & parentBody() const
  {
    return Device::parent();
  }

  /** Return the transformation from the parent body to the display */
  inline const sva::PTransformd & X_b_s() const
  {
    return Device::X_p_s();
  }

  // #################################
  // METHODS RELATED TO SENSOR SIGNALS => https://github.com/frankaemika/libfranka/blob/master/include/franka/vacuum_gripper_state.h
  // #################################

  /** Check if the vacuum value is within in setpoint area */
  inline const bool & get_in_control_range() const
  {
    return in_control_range_;
  }

  inline void set_in_control_range(bool in_control_range)
  {
    in_control_range_ = in_control_range;
  }

  /** Check if part has been detached after a suction cycle */
  inline const bool & get_part_detached() const
  {
    return part_detached_;
  }

  inline void set_part_detached(bool part_detached)
  {
    part_detached_ = part_detached;
  }
  
  /** Check if vacuum is over H2 and not yet under H2-h2 */
  inline const bool & get_part_present() const
  {
    return part_present_;
  }
  
  inline void set_part_present(bool part_present)
  {
    part_present_ = part_present;
  }

  /** Return the current vacuum gripper device status */
  inline const bool & get_device_status_ok() const
  {
    return device_status_ok_;
  }

  inline void set_device_status_ok(bool device_status_ok)
  {
    device_status_ok_ = device_status_ok;
  }

  /** Return the current vacuum gripper actual power. Unit: \f$[%]\f$ */
  inline const uint16_t & get_actual_power() const
  {
    return actual_power_;
  }

  inline void set_actual_power(uint16_t actual_power)
  {
    actual_power_ = actual_power;
  }

  /** Return the current system vacuum. Unit: \f$[mbar]\f$ */
  inline const uint16_t & get_vacuum() const
  {
    return vacuum_;
  }

  inline void set_vacuum(uint16_t vacuum)
  {
    vacuum_ = vacuum;
  }

  // ####################################
  // METHODS RELATED TO ACTUATOR COMMANDS => https://github.com/frankaemika/libfranka/blob/master/include/franka/vacuum_gripper.h
  // ####################################

  /** Vacuum an object. Return true if the vacuum has been established, otherwise return false.
   * @param[in] vacuum Setpoint for control mode. Unit: \f$[10*mbar]\f$.
   * @param[in] timeout Vacuum timeout. Unit: \f$[ms]\f$.
   */
  inline void requestVacuumCommand(const uint8_t vacuum, const std::chrono::milliseconds timeout)
  {
    vacuumCommand_ = vacuum;
    vacuumTimeoutCommand_ = timeout;
    vacuumCommandRequested_ = true;
    // return vacuumSuccessful_;
  }

  inline const bool & vacuumCommandRequested() const
  {
    return vacuumCommandRequested_;
  }

  inline const void getVacuumCommandParams(uint8_t & vacuum, std::chrono::milliseconds & timeout) const
  {
    vacuum = vacuumCommand_;
    timeout = vacuumTimeoutCommand_;
  }

  inline void setVacuumCommandResult(const bool ok)
  {
    vacuumSuccessful_ = ok;
  }

  /** Drop the grasped object off. Return true if command was successful, otherwise return false.
   * @param[in] timeout Dropoff timeout. Unit: \f$[ms]\f$.
   */
  inline void requestDropoffCommand(const std::chrono::milliseconds timeout)
  {
    dropoffTimeoutCommand_ = timeout;
    dropoffCommandRequested_ = true;
    // return dropoffSuccessful_;
  }

  inline const bool & dropoffCommandRequested() const
  {
    return dropoffCommandRequested_;
  }

  inline const void getDropoffCommandParam(std::chrono::milliseconds & timeout) const
  {
    timeout = dropoffTimeoutCommand_;
  }

  inline void setDropoffCommandResult(const bool ok)
  {
    dropoffSuccessful_ = ok;
  }

  /** Stop a currently running vacuum or drop off operation. Return true if command was successful, otherwise return false.*/
  inline void requestStopCommand()
  {
    stopCommandRequested_ = true;
    // return stopSuccessful_;
  }

  inline const bool & stopCommandRequested() const
  {
    return stopCommandRequested_;
  }

  inline void setStopCommandResult(const bool ok)
  {
    stopSuccessful_ = ok;
  }

  mc_rbdyn::DevicePtr clone() const override;

private:
  //sensor signal related members
  bool in_control_range_; // Vacuum value within in setpoint area.
  bool part_detached_; // The part has been detached after a suction cycle.
  bool part_present_; // Vacuum is over H2 and not yet under H2-h2. For more information check the cobot-pump manual.
  bool device_status_ok_; //Current vacuum gripper device status.
  uint16_t actual_power_; // Current vacuum gripper actual power. Unit: \f$[%]\f$.
  uint16_t vacuum_; // Current system vacuum. Unit: \f$[mbar]\f$.

  //actuator command related members
  uint8_t vacuumCommand_ = 0; // Setpoint for control mode. Unit: \f$[10*mbar]\f$.
  std::chrono::milliseconds vacuumTimeoutCommand_ = std::chrono::milliseconds(0); // Vacuum timeout. Unit: \f$[ms]\f$.
  std::chrono::milliseconds dropoffTimeoutCommand_ = std::chrono::milliseconds(0); // Dropoff timeout. Unit: \f$[ms]\f$.
  bool vacuumCommandRequested_ = false;
  bool dropoffCommandRequested_ = false;
  bool stopCommandRequested_ = false;
  bool vacuumSuccessful_ = false;
  bool dropoffSuccessful_ = false;
  bool stopSuccessful_ = false;
};

typedef std::vector<Pump, Eigen::aligned_allocator<Pump>> PumpVector;

} // namespace mc_panda
