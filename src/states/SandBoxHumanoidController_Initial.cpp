#include "SandBoxHumanoidController_Initial.h"

#include "../SandBoxHumanoidController.h"

void SandBoxHumanoidController_Initial::configure(const mc_rtc::Configuration & config) {}

void SandBoxHumanoidController_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<SandBoxHumanoidController &>(ctl_);
}

bool SandBoxHumanoidController_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<SandBoxHumanoidController &>(ctl_);
  output("OK");
  return true;
}

void SandBoxHumanoidController_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<SandBoxHumanoidController &>(ctl_);
}

EXPORT_SINGLE_STATE("SandBoxHumanoidController_Initial", SandBoxHumanoidController_Initial)
