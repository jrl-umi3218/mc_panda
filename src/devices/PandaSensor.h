#pragma once

#include <mc_rbdyn/Device.h>
// #include <mc_rtc/logging.h>
#include <mc_rtc/log/Logger.h>
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
    tau_ext_hat_filtered_ = Eigen::Matrix<double, 7, 1>::Zero();
    O_F_ext_hat_K_ = Eigen::Matrix<double, 6, 1>::Zero();
    K_F_ext_hat_K_ = Eigen::Matrix<double, 6, 1>::Zero();
    joint_contact_ = Eigen::Matrix<double, 7, 1>::Zero();
    cartesian_contact_ = Eigen::Matrix<double, 6, 1>::Zero();
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
  inline const Eigen::Matrix<double, 7, 1> & get_tau_ext_hat_filtered() const
  {
    return tau_ext_hat_filtered_;
  }

  inline void set_tau_ext_hat_filtered(std::array<double, 7> tau_ext_hat_filtered)
  {
    tau_ext_hat_filtered_ = Eigen::Matrix<double, 7, 1>(tau_ext_hat_filtered.data());
  }

  /** Return the estimated external wrench (moment,force) expressed relative to the base frame */
  inline const Eigen::Matrix<double, 6, 1> & get_O_F_ext_hat_K() const
  {
    return O_F_ext_hat_K_;
  }

  inline void set_O_F_ext_hat_K(std::array<double, 6> O_F_ext_hat_K)
  {
    //swap (force,moment) -> (moment,force)
    O_F_ext_hat_K_[3] = O_F_ext_hat_K[0];
    O_F_ext_hat_K_[4] = O_F_ext_hat_K[1];
    O_F_ext_hat_K_[5] = O_F_ext_hat_K[2];
    O_F_ext_hat_K_[0] = O_F_ext_hat_K[3];
    O_F_ext_hat_K_[1] = O_F_ext_hat_K[4];
    O_F_ext_hat_K_[2] = O_F_ext_hat_K[5];
  }

  /** Return the estimated external wrench (moment,force) expressed relative to the stiffness frame*/
  inline const Eigen::Matrix<double, 6, 1> & get_K_F_ext_hat_K() const
  {
    return K_F_ext_hat_K_;
  }

  inline void set_K_F_ext_hat_K(std::array<double, 6> K_F_ext_hat_K)
  {
    //swap (force,moment) -> (moment,force)
    K_F_ext_hat_K_[3] = K_F_ext_hat_K[0];
    K_F_ext_hat_K_[4] = K_F_ext_hat_K[1];
    K_F_ext_hat_K_[5] = K_F_ext_hat_K[2];
    K_F_ext_hat_K_[0] = K_F_ext_hat_K[3];
    K_F_ext_hat_K_[1] = K_F_ext_hat_K[4];
    K_F_ext_hat_K_[2] = K_F_ext_hat_K[5];
  }

  /** Return the control command success rate */
  inline const double get_control_command_success_rate() const
  {
    return control_command_success_rate_;
  }

  inline void set_control_command_success_rate(double control_command_success_rate)
  {
    control_command_success_rate_ = control_command_success_rate;
  }

  /** Return the configured mass of the end effector */
  inline const double get_m_ee() const
  {
    return m_ee_;
  }

  inline void set_m_ee(double m_ee)
  {
    m_ee_ = m_ee;
  }

  /** Return the configured mass of the external load */
  inline const double get_m_load() const
  {
    return m_load_;
  }

  inline void set_m_load(double m_load)
  {
    m_load_ = m_load;
  }

  /** Return which contact level is activated in which joint */
  inline const Eigen::Matrix<double, 7, 1> & get_joint_contact() const
  {
    return joint_contact_;
  }

  inline void set_joint_contact(std::array<double, 7> joint_contact)
  {
    joint_contact_ = Eigen::Matrix<double, 7, 1>(joint_contact_.data());
  }

  /** Return which contact level is activated in which Cartesian dimension (R,P,Y,x,y,z) */
  inline const Eigen::Matrix<double, 6, 1> & get_cartesian_contact() const
  {
    return cartesian_contact_;
  }

  inline void set_cartesian_contact(std::array<double, 6> cartesian_contact)
  {
    //swap (x,y,z,R,P,Y) -> (R,P,Y,x,y,z)
    cartesian_contact_[3] = cartesian_contact[0];
    cartesian_contact_[4] = cartesian_contact[1];
    cartesian_contact_[5] = cartesian_contact[2];
    cartesian_contact_[0] = cartesian_contact[3];
    cartesian_contact_[1] = cartesian_contact[4];
    cartesian_contact_[2] = cartesian_contact[5];
  }

  /** Return singular values */
  inline const Eigen::Matrix<double, 6, 1> & get_singular_values() const
  {
    return singular_values_;
  }

  inline void set_singular_values(Eigen::Matrix<double, 6, 1> singular_values)
  {
    singular_values_ = singular_values;
  }

  // ####################################
  // METHODS RELATED TO ACTUATOR COMMANDS
  // ####################################

  /** Stop the robot */
  inline void requestStopCommand()
  {
    stopRequested_ = true;
  }

  inline const bool & stopRequested() const
  {
    return stopRequested_;
  }

  // ####################################
  // LOGGING
  // ####################################

  inline void addToLogger(mc_rtc::Logger & logger)
  {
    logging=true;
    std::string logname = "PandaSensor_";
    logger.addLogEntry(logname + "tauexthatfiltered", 
      [this]() {
        return (Eigen::VectorXd) tau_ext_hat_filtered_; 
      }
    );
    logger.addLogEntry(logname + "OFexthatK", 
      [this]() {
        return (Eigen::VectorXd) O_F_ext_hat_K_; 
      }
    );
    logger.addLogEntry(logname + "KFexthatK", 
      [this]() {
        return (Eigen::VectorXd) K_F_ext_hat_K_; 
      }
    );
    logger.addLogEntry(logname + "successrate", 
      [this]() {
        return (double) control_command_success_rate_; 
      }
    );
    logger.addLogEntry(logname + "jointcontact", 
      [this]() {
        return (Eigen::VectorXd) joint_contact_; 
      }
    );
    logger.addLogEntry(logname + "cartesiancontact", 
      [this]() {
        return (Eigen::VectorXd) cartesian_contact_; 
      }
    );
    logger.addLogEntry(logname + "singularvalues", 
      [this]() {
        return (Eigen::VectorXd) singular_values_; 
      }
    );
    mc_rtc::log::info("PandaSensor device started to log data"); //TODO
  }
  
  inline void removeFromLogger(mc_rtc::Logger & logger)
  {
    logging=false;
    std::string logname = "PandaSensor_";
    logger.removeLogEntry(logname + "tauexthatfiltered");
    logger.removeLogEntry(logname + "OFexthatK");
    logger.removeLogEntry(logname + "successrate");
    logger.removeLogEntry(logname + "jointcontact");
    logger.removeLogEntry(logname + "cartesiancontact");
  }

  mc_rbdyn::DevicePtr clone() const override;

private:
  bool logging=false;
  //sensor signal related members
  Eigen::Matrix<double, 7, 1> tau_ext_hat_filtered_; //External torque, filtered. Unit: \f$[Nm]\f$.
  Eigen::Matrix<double, 6, 1> O_F_ext_hat_K_; //Estimated external wrench (force, torque) acting on stiffness frame, expressed relative to the base frame. Unit: \f$[N,N,N,Nm,Nm,Nm]\f$.
  Eigen::Matrix<double, 6, 1> K_F_ext_hat_K_; //Estimated external wrench (force, torque) acting on stiffness frame, expressed relative to the stiffness frame. Unit: \f$[N,N,N,Nm,Nm,Nm]\f$.
  double control_command_success_rate_ = 1.0;
  double m_ee_ = 0; //Configured mass of the end effector.
  double m_load_ = 0; //Configured mass of the external load.
  Eigen::Matrix<double, 7, 1> joint_contact_; //Indicates which contact level is activated in which joint. After contact disappears, value turns to zero.
  Eigen::Matrix<double, 6, 1> cartesian_contact_; //Indicates which contact level is activated in which Cartesian dimension (x,y,z,R,P,Y). After contact disappears, the value turns to zero.
  Eigen::Matrix<double, 6, 1> singular_values_;

  //actuator command related members
  bool stopRequested_=false;
};

typedef std::vector<PandaSensor, Eigen::aligned_allocator<PandaSensor>> PandaSensorVector;

} // namespace mc_panda
