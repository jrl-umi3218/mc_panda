#include "PandaSensor.h"

namespace mc_panda
{

PandaSensor::~PandaSensor() = default;

mc_rbdyn::DevicePtr PandaSensor::clone() const
{
  auto device = new PandaSensor(name_, parent_, X_p_d());
  // do extra-copy of required fields

  device->logging=logging;

  //sensor signal related members
  device->tau_ext_hat_filtered_=tau_ext_hat_filtered_;
  device->O_F_ext_hat_K_=O_F_ext_hat_K_;
  device->K_F_ext_hat_K_=K_F_ext_hat_K_;
  device->control_command_success_rate_=control_command_success_rate_;
  device->m_ee_=m_ee_;
  device->m_load_=m_load_;
  device->joint_contact_=joint_contact_;
  device->cartesian_contact_=cartesian_contact_;
  device->singular_values_=singular_values_;
  device->joint_positions_=joint_positions_;
  device->joint_velocities_=joint_velocities_;
  device->joint_torques_=joint_torques_;
  device->joint_dtorques_=joint_dtorques_;

  //actuator command related members
  device->stopRequested_=stopRequested_;

  return mc_rbdyn::DevicePtr(device);
}

} // namespace mc_panda
