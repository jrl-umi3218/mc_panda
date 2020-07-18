#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_control/mc_controller.h>
// #include <mc_rtc/logging.h>
#include <mc_rtc/log/Logger.h>
#include "../devices/PandaSensor.h"
#include "../devices/Pump.h"
#include <Eigen/Core>
#include <Eigen/Eigen>

struct MC_CONTROL_DLLAPI PandaController : public mc_control::fsm::Controller
{
  PandaController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);
  bool run() override;
  void reset(const mc_control::ControllerResetData & reset_data) override;


  /** Check if panda-sensor device is available */
  inline const bool & checkSensorAvailable() const
  {
    return sensorAvailable;
  }

  /** Check if pump device is available */
  inline const bool & checkPumpAvailable() const
  {
    return pumpAvailable;
  }
  

private:
    bool sensorAvailable = false;
    std::shared_ptr<mc_panda::PandaSensor> sensor;
    std::string sensorDeviceName = "PandaSensor";
    bool pumpAvailable = false;
    std::shared_ptr<mc_panda::Pump> pump;
    std::string pumpDeviceName = "Pump";

    Eigen::Matrix<double, 7, 1> tau_ext_hat_filtered_; //External torque, filtered. Unit: \f$[Nm]\f$.
    Eigen::Matrix<double, 6, 1> O_F_ext_hat_K_; //Estimated external wrench (force, torque) acting on stiffness frame, expressed relative to the base frame. Unit: \f$[N,N,N,Nm,Nm,Nm]\f$.
    double control_command_success_rate_ = 0;
    double m_ee_ = 0; //Configured mass of the end effector.
    double m_load_ = 0; //Configured mass of the external load.
    Eigen::Matrix<double, 7, 1> joint_contact_; //Indicates which contact level is activated in which joint. After contact disappears, value turns to zero.
    Eigen::Matrix<double, 6, 1> cartesian_contact_; //Indicates which contact level is activated in which Cartesian dimension (x,y,z,R,P,Y). After contact disappears, the value turns to zero.

    bool in_control_range_ = false; // Vacuum value within in setpoint area.
    bool part_detached_ = false; // The part has been detached after a suction cycle.
    bool part_present_ = false; // Vacuum is over H2 and not yet under H2-h2. For more information check the cobot-pump manual.
    bool device_status_ok_ = false; //Current vacuum gripper device status.
    uint16_t actual_power_ = 0; // Current vacuum gripper actual power. Unit: \f$[%]\f$.
    uint16_t vacuum_ = 0; // Current system vacuum. Unit: \f$[mbar]\f$.
};
