#include "SandBoxHumanoidController_Initial.h"

#include "../SandBoxHumanoidController.h"

void SandBoxHumanoidController_Initial::configure(const mc_rtc::Configuration & config) {}

void SandBoxHumanoidController_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<SandBoxHumanoidController &>(ctl_);
  // Enable feedback from external forces estimator
  if (!ctl.datastore().call<bool>("EF_Estimator::isActive")) {
    ctl.datastore().call("EF_Estimator::toggleActive");
  }
  ctl.tasksComputation(ctl.q_zero_vector);
  ctl.torqueTask->target(ctl.torqueTarget);
  ctl.solver().addTask(ctl.torqueTask);
}

bool SandBoxHumanoidController_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<SandBoxHumanoidController &>(ctl_);
  // output("OK");
   ctl.tasksComputation(ctl.q_zero_vector);
  ctl.torqueTask->target(ctl.torqueTarget);
  return false;
}

void SandBoxHumanoidController_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<SandBoxHumanoidController &>(ctl_);
  ctl.solver().removeTask(ctl.torqueTask);
}

EXPORT_SINGLE_STATE("SandBoxHumanoidController_Initial", SandBoxHumanoidController_Initial)
