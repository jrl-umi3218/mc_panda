#pragma once

#include <mc_rbdyn/Device.h>

#include <vector>

#include <chrono> //required for std::chrono::milliseconds

namespace mc_panda
{

/** This structure defines a PandaSensor vacuum gripper device equipped with a suction cup, 
 * that is a device that can vacuum and blowoff objects */
struct MC_RBDYN_DLLAPI PandaSensor : public mc_rbdyn::Device
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /** Default constructor, does not represent a valid PandaSensor */
  inline PandaSensor() : PandaSensor("", "", sva::PTransformd::Identity()) {}

  /** Constructor
   *
   * @param name Name of the PandaSensor
   *
   * @param bodyName Name of the body to which the PandaSensor is attached
   *
   * @param X_b_s Transformation from the parent body to the PandaSensor
   *
   */
  inline PandaSensor(const std::string & name, const std::string & bodyName, const sva::PTransformd & X_b_s)
  : mc_rbdyn::Device(name, bodyName, X_b_s)
  {
    type_ = "PandaSensor";
  }

  ~PandaSensor() override;

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

  /** Return the external torque */
  inline const std::array<double, 7> & get_tau_ext_hat_filtered() const
  {
    return tau_ext_hat_filtered_;
  }

  inline void set_tau_ext_hat_filtered(std::array<double, 7> tau_ext_hat_filtered)
  {
    tau_ext_hat_filtered_ = tau_ext_hat_filtered;
  }

  /** Return the estimated external wrench */
  inline const std::array<double, 6> & get_O_F_ext_hat_K() const
  {
    return O_F_ext_hat_K_;
  }

  inline void set_O_F_ext_hat_K(std::array<double, 6> O_F_ext_hat_K)
  {
    O_F_ext_hat_K_ = O_F_ext_hat_K;
  }

  mc_rbdyn::DevicePtr clone() const override;

private:
  //sensor signal related members
  std::array<double, 7> tau_ext_hat_filtered_; //External torque, filtered. Unit: \f$[Nm]\f$.
  std::array<double, 6> O_F_ext_hat_K_; //Estimated external wrench (force, torque) acting on stiffness frame, expressed relative to the base frame. Unit: \f$[N,N,N,Nm,Nm,Nm]\f$.
};

typedef std::vector<PandaSensor, Eigen::aligned_allocator<PandaSensor>> PandaSensorVector;

} // namespace mc_panda
