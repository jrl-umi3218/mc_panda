#include "Pump.h"

namespace mc_panda
{

Pump::~Pump() = default;

mc_rbdyn::DevicePtr Pump::clone() const
{
  return mc_rbdyn::DevicePtr(new Pump(*this));
}

} // namespace mc_panda
