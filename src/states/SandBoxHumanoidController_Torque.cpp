#include "SandBoxHumanoidController_Torque.h"

#include "../SandBoxHumanoidController.h"

void SandBoxHumanoidController_Torque::configure(const mc_rtc::Configuration & config) {}

void SandBoxHumanoidController_Torque::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<SandBoxHumanoidController &>(ctl_);
  ctl.solver().addTask(ctl.torqueJointTask);
  ctl.activateTorqueControl(true);
}

bool SandBoxHumanoidController_Torque::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<SandBoxHumanoidController &>(ctl_);
  return false;
}

void SandBoxHumanoidController_Torque::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<SandBoxHumanoidController &>(ctl_);
  ctl.solver().removeTask(ctl.torqueJointTask);
}

EXPORT_SINGLE_STATE("SandBoxHumanoidController_Torque", SandBoxHumanoidController_Torque)
