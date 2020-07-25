#include "Pump.h"

namespace mc_panda
{

Pump::~Pump() = default;

mc_rbdyn::DevicePtr Pump::clone() const
{
  auto device = new Pump(name_, parent_, X_p_d());
  // do extra-copy of required fields

  //sensor signal related members
  device->in_control_range_=in_control_range_;
  device->part_detached_=part_detached_;
  device->part_present_=part_present_;
  device->device_status_ok_=device_status_ok_;
  device->actual_power_=actual_power_;
  device->vacuum_=vacuum_;

  //actuator command related members
  device->vacuumCommand_=vacuumCommand_;
  device->vacuumTimeoutCommand_=vacuumTimeoutCommand_;
  device->dropoffTimeoutCommand_=dropoffTimeoutCommand_;
  device->vacuumSuccessful_=vacuumSuccessful_;
  device->dropoffSuccessful_=dropoffSuccessful_;
  device->stopSuccessful_=stopSuccessful_;
  device->nc=nc;

  return mc_rbdyn::DevicePtr(device);
}

} // namespace mc_panda
