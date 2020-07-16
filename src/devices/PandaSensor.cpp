#include "PandaSensor.h"

namespace mc_panda
{

PandaSensor::~PandaSensor() = default;

mc_rbdyn::DevicePtr PandaSensor::clone() const
{
  return mc_rbdyn::DevicePtr(new PandaSensor(*this));
}

} // namespace mc_panda
