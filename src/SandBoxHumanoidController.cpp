#include "SandBoxHumanoidController.h"
#include <Eigen/src/Core/Matrix.h>

SandBoxHumanoidController::SandBoxHumanoidController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::TVM)
{
  //Initialize Constraints
  selfCollisionConstraint->setCollisionsDampers(solver(), {1.2, 50.0});
  solver().removeConstraintSet(dynamicsConstraint);
  dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
    new mc_solver::DynamicsConstraint(robots(), 0, {0.1, 0.01, 0.0, 1.2, 200.0}, 0.8, true));
  solver().addConstraintSet(dynamicsConstraint);

  torqueTask = std::make_shared<mc_tasks::TorqueTask>(solver(), robot().robotIndex());

  // Get the default posture target from the robot's posture task
  std::shared_ptr<mc_tasks::PostureTask> FSMPostureTask = getPostureTask(robot().name());
  auto posture = FSMPostureTask->posture();
  solver().removeTask(FSMPostureTask);
  size_t i = 0;

  std::map<std::string, double> kp = config("kp");
  std::map<std::string, double> kd = config("kd");

  for (const auto &j : robot().mb().joints()) 
  {
    const std::string &joint_name = j.name();
    if(j.type() == rbd::Joint::Type::Rev)
    {
      jointNames.emplace_back(joint_name);  
      if (const auto &t = posture[robot().jointIndexByName(joint_name)]; !t.empty()) 
      {
        kp_vector[i] = kp.at(joint_name);
        kd_vector[i] = kd.at(joint_name);
        q_zero_vector[i] = t[0];
        torqueTarget[joint_name] = {0.0};
        mc_rtc::log::info("[RLController] Joint {}: currentTargetPosition {}, kp {}, kd {}", joint_name, q_zero_vector[i], kp_vector[i], kd_vector[i]);
        i++;
      }
    }
  }

  if(!datastore().has("anchorFrameFunction"))
  {
    datastore().make_call("anchorFrameFunction", [this](const mc_rbdyn::Robot & real_robot) {return createContactAnchor(real_robot);});
  }

  datastore().make<std::string>("ControlMode", "Torque");
  addGui();

  mc_rtc::log::success("SandBoxHumanoidController init done.");
}

bool SandBoxHumanoidController::run()
{
  return mc_control::fsm::Controller::run(mc_solver::FeedbackType::ClosedLoopIntegrateReal);
}

void SandBoxHumanoidController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}


void SandBoxHumanoidController::tasksComputation(Eigen::VectorXd & currentTargetPosition)
{
  auto & robot = robots()[0];
  auto & real_robot = realRobot(robots()[0].name());

  auto q = real_robot.encoderValues();
  Eigen::VectorXd currentPos = Eigen::VectorXd::Map(q.data(), q.size());
  auto vel = real_robot.encoderVelocities();
  Eigen::VectorXd currentVel = Eigen::VectorXd::Map(vel.data(), vel.size());
  auto tau = real_robot.jointTorques();

  Eigen::VectorXd tau_d = (kp_vector).cwiseProduct(currentTargetPosition - currentPos) - (kd_vector).cwiseProduct(currentVel);
   
  size_t i = 0;
  for (const auto &joint_name : jointNames)
  {
    torqueTarget[joint_name][0] = tau_d[i];
    i++;
  }
}

std::pair<sva::PTransformd, Eigen::Vector3d> SandBoxHumanoidController::createContactAnchor(const mc_rbdyn::Robot & anchorRobot)
{
  sva::PTransformd X_foot_r = anchorRobot.bodyPosW("right_ankle_link");
  sva::PTransformd X_foot_l = anchorRobot.bodyPosW("left_ankle_link");

  sva::MotionVecd v_foot_r = anchorRobot.bodyVelW("right_ankle_link");
  sva::MotionVecd v_foot_l = anchorRobot.bodyVelW("left_ankle_link");

  auto extTorqueSensor = robot().device<mc_rbdyn::VirtualTorqueSensor>("ExtTorquesVirtSensor");
  int right_knee_index = robot().jointIndexByName("right_knee_joint") + 5;
  int left_knee_index = robot().jointIndexByName("left_knee_joint") + 5;
  double tau_ext_knee_r =  abs(extTorqueSensor.torques()[right_knee_index]);
  double tau_ext_knee_l =  abs(extTorqueSensor.torques()[left_knee_index]);
  double leftFootRatio = tau_ext_knee_l/(tau_ext_knee_r+tau_ext_knee_l);
  if(tau_ext_knee_r + tau_ext_knee_l < 0.02)
  {
    leftFootRatio = 0.5;
  }
         
  Eigen::VectorXd w_r = X_foot_r.translation();
  Eigen::VectorXd w_l = X_foot_l.translation();
  Eigen::VectorXd contact_anchor = (w_r * (1 - leftFootRatio) + w_l * leftFootRatio)  ;
  Eigen::VectorXd anchor_vel = (v_foot_r.linear() * (1 - leftFootRatio) + v_foot_l.linear() * leftFootRatio);
  sva::PTransformd contact_anchor_tf = sva::PTransformd(Eigen::Matrix3d::Identity(), contact_anchor); 

  return {contact_anchor_tf, anchor_vel};
}

void SandBoxHumanoidController::addGui(void)
{
  gui()->addElement({"SandBoxHumanoidController", "Gains"},
    mc_rtc::gui::NumberInput("kp legs", [this]() { return kp_vector; },
      [this](double v) { 
        kp_vector.segment(0, 4).setConstant(v); 
        kp_vector.segment(5, 4).setConstant(v); 
      })
  );

  gui()->addElement({"SandBoxHumanoidController", "Gains"},
    mc_rtc::gui::NumberInput("kp ankles", [this]() { return kp_vector; },
      [this](double v) { 
        kp_vector(4) = v; 
        kp_vector(9) = v; 
      })
  );

  gui()->addElement({"SandBoxHumanoidController", "Gains"},
    mc_rtc::gui::NumberInput("kp torso", [this]() { return kp_vector; },
      [this](double v) { 
        kp_vector(10) = v; 
      })
  );

  gui()->addElement({"SandBoxHumanoidController", "Gains"},
    mc_rtc::gui::NumberInput("kp arms", [this]() { return kp_vector; },
      [this](double v) { 
        kp_vector.segment(11, 4).setConstant(v); 
        kp_vector.segment(15, 4).setConstant(v); 
      })
  );

  gui()->addElement({"SandBoxHumanoidController", "Gains"},
    mc_rtc::gui::NumberInput("kd legs", [this]() { return kd_vector; },
      [this](double v) { 
        kd_vector.segment(0, 4).setConstant(v); 
        kd_vector.segment(5, 4).setConstant(v); 
      })
  );

  gui()->addElement({"SandBoxHumanoidController", "Gains"},
    mc_rtc::gui::NumberInput("kd ankles", [this]() { return kd_vector; },
      [this](double v) { 
        kd_vector(4) = v; 
        kd_vector(9) = v; 
      })
  );

  gui()->addElement({"SandBoxHumanoidController", "Gains"},
    mc_rtc::gui::NumberInput("kd torso", [this]() { return kd_vector; },
      [this](double v) { 
        kd_vector(10) = v; 
      })
  );

  gui()->addElement({"SandBoxHumanoidController", "Gains"},
    mc_rtc::gui::NumberInput("kd arms", [this]() { return kd_vector; },
      [this](double v) { 
        kd_vector.segment(11, 4).setConstant(v); 
        kd_vector.segment(15, 4).setConstant(v); 
      })
  );
}