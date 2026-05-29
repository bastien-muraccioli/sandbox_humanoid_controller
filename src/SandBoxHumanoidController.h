#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_tasks/PostureTask.h>
#include <mc_tasks/TorqueJointTask.h>

#include "api.h"

struct SandBoxHumanoidController_DLLAPI SandBoxHumanoidController : public mc_control::fsm::Controller
{
  SandBoxHumanoidController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

  std::shared_ptr<mc_tasks::TorqueJointTask> torqueJointTask;
  std::shared_ptr<mc_tasks::PostureTask> postureTask;

  void activateTorqueControl(bool activate);

private:
  void addGui();
  void addLog();
  void computeLimits();
  void updateExternalTorque();
  
  mc_rtc::Configuration config_;
  double ds_ = 0.0;
  double velocityLimitPercent_ = 0.99;
  
  bool controlModeChanged_ = false;
  bool isTorqueControl_ = false;

  bool contactModeChanged_ = true;
  bool contactConstraintsAreEnabled_ = true;

  double gainRatio_ = 0.5;
  Eigen::VectorXd kpVector_;
  Eigen::VectorXd kdVector_;
  Eigen::VectorXd externalTorques_;
  bool applyExternalTorque_ = false;
};
