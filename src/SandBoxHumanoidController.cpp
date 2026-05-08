#include "SandBoxHumanoidController.h"
#include <mc_rtc/gui/ArrayLabel.h>
#include <mc_rtc/gui/NumberSlider.h>
#include <mc_solver/QPSolver.h>
#include <Eigen/src/Core/Matrix.h>

SandBoxHumanoidController::SandBoxHumanoidController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::TVM)
{
  // Initialize Constraints
  selfCollisionConstraint->setCollisionsDampers(solver(), {1.2, 100.0});
  solver().removeConstraintSet(dynamicsConstraint);
  dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
    new mc_solver::DynamicsConstraint(robots(), 0, {0.1, 0.01, 0.0, 1.2, 100.0}, 0.99, true));
  solver().addConstraintSet(dynamicsConstraint);

  // Get the default posture target from the robot's posture task
  std::shared_ptr<mc_tasks::PostureTask> FSMPostureTask = getPostureTask(robot().name());
  auto posture = FSMPostureTask->posture();
  solver().removeTask(FSMPostureTask);
  

  std::map<std::string, double> kp = config(robot().name())("kp");
  std::map<std::string, double> kd = config(robot().name())("kd");

  Eigen::VectorXd kp_vector = Eigen::VectorXd::Zero(robot().mb().nrDof() - 6);
  Eigen::VectorXd kd_vector = Eigen::VectorXd::Zero(robot().mb().nrDof() - 6);
  
  size_t i = 0;
  for (const auto &j : robot().mb().joints()) 
  {
    const std::string &joint_name = j.name();
    if(j.type() == rbd::Joint::Type::Rev)
    {
      if (const auto &t = posture[robot().jointIndexByName(joint_name)]; !t.empty()) 
      {
        kp_vector[i] = kp.at(joint_name);
        kd_vector[i] = kd.at(joint_name);
        mc_rtc::log::info("[SandBoxHumanoidController] Joint {}: kp {}, kd {}", joint_name, kp_vector[i], kd_vector[i]);
        i++;
      }
    }
  }

  torqueJointTask = std::make_shared<mc_tasks::TorqueJointTask>(solver(), robot().robotIndex(), 10.0, 10.0);
  torqueJointTask->setStiffness(kp_vector);
  torqueJointTask->setDamping(kd_vector);
  solver().addTask(torqueJointTask);

  datastore().make<std::string>("ControlMode", "Torque");

  // Eigen::Vector6d footcontact_dof = Eigen::Vector6d(0, 0, 1, 0, 0, 0);
  // addContact({robot().name(), "ground", "RightFootCenter", "AllGround", 0.7, footcontact_dof});
  // addContact({robot().name(), "ground", "LeftFootCenter", "AllGround", 0.7, footcontact_dof});

  mc_rtc::log::success("[SandBoxHumanoidController] init done.");
}

bool SandBoxHumanoidController::run()
{
  return mc_control::fsm::Controller::run(mc_solver::FeedbackType::ClosedLoopIntegrateReal);
  // return mc_control::fsm::Controller::run();
}

void SandBoxHumanoidController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}