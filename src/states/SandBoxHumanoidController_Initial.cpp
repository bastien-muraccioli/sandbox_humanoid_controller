#include "SandBoxHumanoidController_Initial.h"

#include "../SandBoxHumanoidController.h"

void SandBoxHumanoidController_Initial::configure(const mc_rtc::Configuration & config) {}

void SandBoxHumanoidController_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<SandBoxHumanoidController &>(ctl_);
  ctl.solver().addTask(ctl.postureTask);
  ctl.activateTorqueControl(false);
}

bool SandBoxHumanoidController_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<SandBoxHumanoidController &>(ctl_);
  return false;
}

void SandBoxHumanoidController_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<SandBoxHumanoidController &>(ctl_);
  ctl.solver().removeTask(ctl.postureTask);
}

EXPORT_SINGLE_STATE("SandBoxHumanoidController_Initial", SandBoxHumanoidController_Initial)
