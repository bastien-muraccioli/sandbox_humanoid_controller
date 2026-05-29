#include "SandBoxHumanoidController_TorquePos.h"

#include "../SandBoxHumanoidController.h"

void SandBoxHumanoidController_TorquePos::configure(const mc_rtc::Configuration & config) {}

void SandBoxHumanoidController_TorquePos::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<SandBoxHumanoidController &>(ctl_);
  ctl.solver().addTask(ctl.torqueJointTask);
  ctl.activateTorqueControl(false);
}

bool SandBoxHumanoidController_TorquePos::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<SandBoxHumanoidController &>(ctl_);
  return false;
}

void SandBoxHumanoidController_TorquePos::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<SandBoxHumanoidController &>(ctl_);
  ctl.solver().removeTask(ctl.torqueJointTask);
}

EXPORT_SINGLE_STATE("SandBoxHumanoidController_TorquePos", SandBoxHumanoidController_TorquePos)
