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

  //METHODS RELATED TO SENSOR SIGNALS => https://github.com/frankaemika/libfranka/blob/master/include/franka/vacuum_gripper_state.h

  /** Check if the vacuum value is within in setpoint area */
  inline const bool & in_control_range() const
  {
    return in_control_range_;
  }

  /** Check if part has been detached after a suction cycle */
  inline const bool & part_detached() const
  {
    return part_detached_;
  }
  
  /** Check if vacuum is over H2 and not yet under H2-h2 */
  inline const bool & part_present() const
  {
    return part_present_;
  }
  
  /** Return the current vacuum gripper device status */
  inline const uint8_t & device_status() const
  {
    return device_status_;
  }

  /** Return the current vacuum gripper actual power. Unit: \f$[%]\f$ */
  inline const uint16_t & actual_power() const
  {
    return actual_power_;
  }

  /** Return the current system vacuum. Unit: \f$[mbar]\f$ */
  inline const uint16_t & vacuum() const
  {
    return vacuum_;
  }

  //METHODS RELATED TO ACTUATOR COMMANDS => https://github.com/frankaemika/libfranka/blob/master/include/franka/vacuum_gripper.h

  /** Vacuum an object. Return true if the vacuum has been established, otherwise return false.
   * @param[in] vacuum Setpoint for control mode. Unit: \f$[10*mbar]\f$.
   * @param[in] timeout Vacuum timeout. Unit: \f$[ms]\f$.
   */
  inline bool & vacuum(const uint8_t vacuum, const std::chrono::milliseconds timeout)
  {
    return vacuumSuccessful_;
  }

  /** Drop the grasped object off. Return true if command was successful, otherwise return false.
   * @param[in] timeout Dropoff timeout. Unit: \f$[ms]\f$.
   */
  inline bool & dropOff(const std::chrono::milliseconds timeout)
  {
    return dropOffSuccessful_;
  }

  /** Stop a currently running vacuum or drop off operation. Return true if command was successful, otherwise return false.*/
  inline bool & stop()
  {
    return stopSuccessful_;
  }

  mc_rbdyn::DevicePtr clone() const override;

private:
  //sensor signals
  bool in_control_range_; // Vacuum value within in setpoint area.
  bool part_detached_; // The part has been detached after a suction cycle
  bool part_present_; // Vacuum is over H2 and not yet under H2-h2. For more information check the cobot-pump manual.
  uint8_t device_status_; //Current vacuum gripper device status.
  uint16_t actual_power_; // Current vacuum gripper actual power. Unit: \f$[%]\f$.
  uint16_t vacuum_; // Current system vacuum. Unit: \f$[mbar]\f$.

  //actuator command feedback
  bool vacuumSuccessful_;
  bool dropOffSuccessful_;
  bool stopSuccessful_;
};

typedef std::vector<Pump, Eigen::aligned_allocator<Pump>> PumpVector;

} // namespace mc_panda
