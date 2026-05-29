#include "SandBoxHumanoidController.h"
#include <mc_rtc/gui/ArrayLabel.h>
#include <mc_rtc/gui/NumberInput.h>
#include <mc_rtc/gui/Button.h>
#include <mc_rtc/gui/Force.h>
#include <mc_solver/QPSolver.h>
#include <Eigen/src/Core/Matrix.h>
#include <string>
#include <mc_tvm/Robot.h>
#include <RBDyn/MultiBodyConfig.h>

SandBoxHumanoidController::SandBoxHumanoidController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::TVM)
{
  // Initialize Constraints
  selfCollisionConstraint->setCollisionsDampers(solver(), {1.2, 100.0});
  solver().removeConstraintSet(dynamicsConstraint);
  dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
    new mc_solver::DynamicsConstraint(robots(), 0, {0.1, ds_, 0.0, 1.2, 100.0}, velocityLimitPercent_, true));
  solver().addConstraintSet(dynamicsConstraint);
  mc_rtc::log::info("Contact constraint type: {}", static_cast<int>(contactConstraint().contactType()));

  // Get the default posture target from the robot's posture task
  postureTask = getPostureTask(robot().name());
  auto posture = postureTask->posture();
  

  std::map<std::string, double> kp = config(robot().name())("kp");
  std::map<std::string, double> kd = config(robot().name())("kd");

  kpVector_ = Eigen::VectorXd::Zero(robot().mb().nrDof() - 6);
  kdVector_ = Eigen::VectorXd::Zero(robot().mb().nrDof() - 6);

  size_t i = 0;
  for (const auto &j : robot().mb().joints()) 
  {
    const std::string &joint_name = j.name();
    if(j.type() == rbd::Joint::Type::Rev)
    {
      if (const auto &t = posture[robot().jointIndexByName(joint_name)]; !t.empty()) 
      {
        kpVector_[i] = kp.at(joint_name);
        kdVector_[i] = kd.at(joint_name);
        i++;
      }
    }
  }

  torqueJointTask = std::make_shared<mc_tasks::TorqueJointTask>(solver(), robot().robotIndex(), 10.0, 10.0);
  torqueJointTask->setStiffness(gainRatio_ * kpVector_);
  torqueJointTask->setDamping(gainRatio_ * kdVector_);

  datastore().make<std::string>("ControlMode", "Position");

  externalTorques_ = Eigen::VectorXd::Zero(robot().mb().nrDof());

  addGui();
  addLog();

  mc_rtc::log::success("[SandBoxHumanoidController] init done.");
}

bool SandBoxHumanoidController::run()
{
  auto & real_robot = realRobot(robots()[0].name());
  auto & q_mbc = robot().mbc().q;
  auto & q_real_mbc = real_robot.q();
  auto & qdot_mbc = robot().mbc().alpha;
  auto & qdot_real_mbc = real_robot.alpha();

  Eigen::VectorXd q = rbd::sParamToVector(robot().mb(), q_mbc);
  Eigen::VectorXd qdot = rbd::sDofToVector(robot().mb(), qdot_mbc);
  Eigen::VectorXd q_real = rbd::sParamToVector(real_robot.mb(), q_real_mbc);
  Eigen::VectorXd qdot_real = rbd::sDofToVector(real_robot.mb(), qdot_real_mbc);

  // mc_rtc::log::info("Current q floating base: {}", q.head(7).transpose());
  // mc_rtc::log::info("Real robot q floating base: {}", q_real.head(7).transpose());
  // mc_rtc::log::info("Current qdot floating base: {}", qdot.head(6).transpose());
  // mc_rtc::log::info("Real robot qdot floating base: {}", qdot_real.head(6).transpose());

  if(controlModeChanged_)
  {
    if(isTorqueControl_)
    {
      mc_rtc::log::info("Switching to Torque Control");
      datastore().assign<std::string>("ControlMode", "Torque");
    }
    else
    {
      mc_rtc::log::info("Switching to Position Control");
      datastore().assign<std::string>("ControlMode", "Position");
    }
    controlModeChanged_ = false;
  }
  if(contactModeChanged_)
  {
    if(contactConstraintsAreEnabled_)
    {
      Eigen::Vector6d footcontact_dof = Eigen::Vector6d(1, 1, 1, 1, 1, 1);
      addContact({robot().name(), "ground", "RightFootCenter", "AllGround", 0.7, footcontact_dof});
      addContact({robot().name(), "ground", "LeftFootCenter", "AllGround", 0.7, footcontact_dof});
    }
    else
    {
      clearContacts();
    }
    contactModeChanged_ = false;
  }

  updateExternalTorque();
  computeLimits();
  auto ctrl_mode = datastore().get<std::string>("ControlMode");
  bool run;
  if (ctrl_mode.compare("Position") == 0) {
    run= mc_control::fsm::Controller::run();
  } else {
    run= mc_control::fsm::Controller::run(
        mc_solver::FeedbackType::ClosedLoopIntegrateReal);
  }

  return run;
}

void SandBoxHumanoidController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}

void SandBoxHumanoidController::computeLimits()
{
  double epsilon = 1e-5;

  auto & real_robot = realRobot(robots()[0].name());
  auto currentPos = real_robot.q();
  auto currentVel = real_robot.alpha();
  auto currentTau = real_robot.jointTorque();

  auto qLimLower = real_robot.ql();
  auto qLimUpper = real_robot.qu();

  auto qDotLimLower = real_robot.vl();
  auto qDotLimUpper = real_robot.vu();

  auto tauLimLower = real_robot.tl();
  auto tauLimUpper = real_robot.tu();

  for (std::string joint : robot().refJointOrder())
  {
    int i = robot().jointIndexByName(joint);

    double ds = ds_ * (qLimUpper[i][0] - qLimLower[i][0]);
    double posLimitUp = qLimUpper[i][0] - ds;
    double posLimitLow = qLimLower[i][0] + ds;
    double velLimitUp = velocityLimitPercent_ * qDotLimUpper[i][0];
    double velLimitLow = velocityLimitPercent_ * qDotLimLower[i][0];
    double tauLimitUp = tauLimUpper[i][0];
    double tauLimitLow = tauLimLower[i][0];

    if (currentPos[i][0] > posLimitUp + epsilon)
    {
      mc_rtc::log::warning("Joint {} position upper limit breached: currentPos = {}, limit = {}", joint, currentPos[i][0], posLimitUp);
    }
    if (currentPos[i][0] < posLimitLow - epsilon)
    {
      mc_rtc::log::warning("Joint {} position lower limit breached: currentPos = {}, limit = {}", joint, currentPos[i][0], posLimitLow);
    }
    if (currentVel[i][0] > velLimitUp + epsilon)
    {
      mc_rtc::log::warning("Joint {} velocity upper limit breached: currentVel = {}, limit = {}", joint, currentVel[i][0], velLimitUp);
    }
    if (currentVel[i][0] < velLimitLow - epsilon)
    {
      mc_rtc::log::warning("Joint {} velocity lower limit breached: currentVel = {}, limit = {}", joint, currentVel[i][0], velLimitLow);
    }
    if (currentTau[i][0] > tauLimitUp + epsilon)    {
      mc_rtc::log::warning("Joint {} torque upper limit breached: currentTau = {}, limit = {}", joint, currentTau[i][0], tauLimitUp);
    }
    if (currentTau[i][0] < tauLimitLow - epsilon)    {
      mc_rtc::log::warning("Joint {} torque lower limit breached: currentTau = {}, limit = {}", joint, currentTau[i][0], tauLimitLow);
    }
  }
}

void SandBoxHumanoidController::updateExternalTorque()
{
  auto & robot = robots()[0];
  auto & real_robot = realRobot(robots()[0].name());

  // Reset each cycle — never accumulate across control iterations
  externalTorques_ = Eigen::VectorXd::Zero(robot.mb().nrDof());

  for(const auto & ft_sensor : real_robot.forceSensors())
  {
    // Transformation from parent body origin to sensor frame, used to place
    // the Jacobian at the exact sensor location rather than the body origin,
    // ensuring the moment arm is correct
    const sva::PTransformd & X_p_f = ft_sensor.X_p_f();
    auto jac = rbd::Jacobian(real_robot.mb(), ft_sensor.parentBody(), X_p_f.translation());

    // World-frame Jacobian (6 x path_dof), then expanded to full robot DoF
    // so J^T maps a world-frame wrench to all joint torques
    Eigen::MatrixXd shortJac = jac.jacobian(real_robot.mb(), real_robot.mbc());
    Eigen::MatrixXd fullJac = Eigen::MatrixXd::Zero(6, real_robot.mb().nrDof());
    jac.fullJacobian(real_robot.mb(), shortJac, fullJac);

    // wrenchWithoutGravity returns the wrench in the sensor (body) frame.
    // R.transpose() rotates it to the world frame to match the world-frame
    // Jacobian — virtual work requires both to be expressed in the same frame
    const Eigen::Matrix3d & R = real_robot.bodyPosW(ft_sensor.parentBody()).rotation();
    sva::ForceVecd w = ft_sensor.wrenchWithoutGravity(real_robot);
    w.force() = R.transpose() * w.force();
    w.couple() = R.transpose() * w.couple();

    // τ_ext += J^T * F: project the external wrench into joint torque space
    // and accumulate contributions from all sensors
    externalTorques_ += fullJac.transpose() * w.vector();
  }

  externalTorques_ -= dynamicsConstraint->dynamicFunction().contactTorque();
  
  if(applyExternalTorque_)
  {
     robot.setExternalTorques(externalTorques_);
     real_robot.setExternalTorques(externalTorques_);
  }
  else
  {
     robot.setExternalTorques(Eigen::VectorXd::Zero(real_robot.mb().nrDof()));
     real_robot.setExternalTorques(Eigen::VectorXd::Zero(real_robot.mb().nrDof()));
  }
}

void SandBoxHumanoidController::addGui()
{
  gui()->addElement({"SandBoxHumanoidController", "ControlMode"},
      mc_rtc::gui::Button("Toggle Control Mode", [this]()
        {
          isTorqueControl_ = !isTorqueControl_;
          controlModeChanged_ = true;
        }),
      mc_rtc::gui::Label("Current Control Mode", [this]()
        {
          return isTorqueControl_ ? "Torque Control" : "Position Control";
        }),
      mc_rtc::gui::Button("Toggle External Torque", [this]()
        {
          applyExternalTorque_ = !applyExternalTorque_;
        }),
      mc_rtc::gui::Label("External Torque", [this]()
        {
          return applyExternalTorque_ ? "Enabled" : "Disabled";
        }),
      mc_rtc::gui::Button("Toggle Contact constraint", [this]()
        {
          contactConstraintsAreEnabled_ = !contactConstraintsAreEnabled_;
          contactModeChanged_ = true;
        }),
      mc_rtc::gui::Label("Contact constraint", [this]()
        {
          return contactConstraintsAreEnabled_ ? "Enabled" : "Disabled";
        })
    );

  gui()->addElement({"SandBoxHumanoidController", "Ratio"},
    mc_rtc::gui::NumberInput("Gain Ratio", [this]() { return gainRatio_; }, 
    [this](double val) 
    { 
      gainRatio_ = val;
      torqueJointTask->setStiffness(gainRatio_ * kpVector_);
      torqueJointTask->setDamping(gainRatio_ * kdVector_); 
    })
  );

  auto & robot = realRobot(robots()[0].name());
  for(const auto & ft_sensor : robot.forceSensors()) {
    gui()->addElement({"SandBoxHumanoidController", "FT Sensors", ft_sensor.name()},
      mc_rtc::gui::Force(
        ft_sensor.name(), 
        [&ft_sensor, &robot]() { return ft_sensor.wrenchWithoutGravity(robot); }, 
        [&ft_sensor, &robot]() { return robot.bodyPosW(ft_sensor.parent()); })
    );
  }
}

void SandBoxHumanoidController::addLog()
{
  logger().addLogEntry("ExternalTorques", [this]() { return externalTorques_; });
  logger().addLogEntry("isTorqueControl", [this]() { return isTorqueControl_;});
}

void SandBoxHumanoidController::activateTorqueControl(bool activate)
{
  if(activate && !isTorqueControl_)
  {
    isTorqueControl_ = true;
    controlModeChanged_ = true;
  }
  else if(!activate && isTorqueControl_)
  {
    isTorqueControl_ = false;
    controlModeChanged_ = true;
  }
}