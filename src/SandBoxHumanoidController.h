#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_tasks/TorqueTask.h>

#include "api.h"
#include <string>

struct SandBoxHumanoidController_DLLAPI SandBoxHumanoidController : public mc_control::fsm::Controller
{
  SandBoxHumanoidController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

  void tasksComputation(Eigen::VectorXd & currentTargetPosition);

  std::pair<sva::PTransformd, Eigen::Vector3d>  createContactAnchor(const mc_rbdyn::Robot & anchorRobot);
  void addGui(void);

  std::vector<std::string> jointNames;

  std::shared_ptr<mc_tasks::TorqueTask> torqueTask;
  std::map<std::string, std::vector<double>> torqueTarget;

  Eigen::VectorXd q_zero_vector;
  Eigen::VectorXd kp_vector;
  Eigen::VectorXd kd_vector;

private:
  mc_rtc::Configuration config_;
};
