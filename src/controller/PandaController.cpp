#include "PandaController.h"

PandaController::PandaController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{
  LOG_SUCCESS("using the robot " << rm->name)

  LOG_SUCCESS("PandaController init done " << this)
}

bool PandaController::run()
{
  if(sensorAvailable)
  {
    tau_ext_hat_filtered_ = sensor->get_tau_ext_hat_filtered();
    O_F_ext_hat_K_ = sensor->get_O_F_ext_hat_K();
    control_command_success_rate_ = sensor->get_control_command_success_rate();
    m_ee_ = sensor->get_m_ee();
    m_load_ = sensor->get_m_load();
    joint_contact_ = sensor->get_joint_contact();
    cartesian_contact_ = sensor->get_cartesian_contact();
  }

  if(pumpAvailable)
  {
    in_control_range_ = pump->get_in_control_range();
    part_detached_ = pump->get_part_detached();
    part_present_ = pump->get_part_present();
    device_status_ok_ = pump->get_device_status_ok();
    actual_power_ = pump->get_actual_power();
    vacuum_ = pump->get_vacuum();
  }
  return mc_control::fsm::Controller::run();
}

void PandaController::reset(const mc_control::ControllerResetData & reset_data)
{
  // Update dynamics constraints
  dynamicsConstraint = mc_solver::DynamicsConstraint(robots(),
                                                     robot().robotIndex(),
                                                     solver().dt(), {0.1, 0.01, 0.5});
  // Must be added to the solver before controller reset
  solver().addConstraintSet(dynamicsConstraint);
  mc_control::fsm::Controller::reset(reset_data);

  if(robot().hasDevice<mc_panda::PandaSensor>(sensorDeviceName))
  {
    sensor = std::make_shared<mc_panda::PandaSensor>( robot().device<mc_panda::PandaSensor>(sensorDeviceName) );
    sensorAvailable = true;
    mc_rtc::log::info("RobotModule has a PandaSensor named {}", sensorDeviceName);
  }
  else{
    mc_rtc::log::warning("RobotModule does not have a PandaSensor named {}", sensorDeviceName);
    mc_rtc::log::warning("PandaSensor functionality will not be available");
  }

  if(robot().hasDevice<mc_panda::Pump>(pumpDeviceName))
  {
    pump = std::make_shared<mc_panda::Pump>( robot().device<mc_panda::Pump>(pumpDeviceName) );
    pumpAvailable = true;
    mc_rtc::log::info("RobotModule has a Pump named {}", pumpDeviceName);
  }
  else{
    mc_rtc::log::warning("RobotModule does not have a Pump named {}", pumpDeviceName);
    mc_rtc::log::warning("Pump functionality will not be available");
  }

  tau_ext_hat_filtered_ = Eigen::Matrix<double, 7, 1>::Zero();
  O_F_ext_hat_K_ = Eigen::Matrix<double, 6, 1>::Zero();
  joint_contact_ = Eigen::Matrix<double, 7, 1>::Zero();
  cartesian_contact_ = Eigen::Matrix<double, 6, 1>::Zero();

  if(sensorAvailable)
  {
    logger().addLogEntry("successrate", 
      [this]() {
        return (double) control_command_success_rate_; 
      }
    );
    logger().addLogEntry("tauexthatfilteredVector", 
      [this]() {
        return (Eigen::VectorXd) tau_ext_hat_filtered_; 
      }
    );
    logger().addLogEntry("OFexthatKVector", 
      [this]() {
        return (Eigen::VectorXd) O_F_ext_hat_K_; 
      }
    );
    logger().addLogEntry("jointcontactVector", 
      [this]() {
        return (Eigen::VectorXd) joint_contact_; 
      }
    );
    logger().addLogEntry("cartesiancontactVector", 
      [this]() {
        return (Eigen::VectorXd) cartesian_contact_; 
      }
    );
  }
  if(pumpAvailable)
  {
    logger().addLogEntry("status", 
      [this]() {
        return (bool) device_status_ok_; 
      }
    );
    logger().addLogEntry("actualpower", 
      [this]() {
        return (uint16_t) actual_power_; 
      }
    );
    logger().addLogEntry("vacuum", 
      [this]() {
        return (uint16_t) vacuum_; 
      }
    );
  }
}

