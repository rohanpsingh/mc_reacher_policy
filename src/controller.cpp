#include "controller.h"

HandReacher::HandReacher(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)

: mc_control::fsm::Controller(rm, dt, config)
{
  config_.load(config);

  mc_rtc::log::info("Loading default stabilizer configuration");
  // Load default configuration from robot module
  auto stabiConfig = robot().module().defaultLIPMStabilizerConfiguration();

  auto robotConfig = config("robot_models")(controlRobot().name());
  if(robotConfig.has("stabilizer"))
  {
    mc_rtc::log::info("Loading additional stabilizer configuration:\n{}", robotConfig("stabilizer").dump(true));
    stabiConfig.load(robotConfig("stabilizer"));
    mc_rtc::log::info("Stabi prop {}", stabiConfig.dcmPropGain);
  }

  stabilizer_.reset(
    new mc_tasks::lipm_stabilizer::StabilizerTask(
      solver().robots(),
      solver().realRobots(),
      robot().robotIndex(),
      stabiConfig.leftFootSurface,
      stabiConfig.rightFootSurface,
      stabiConfig.torsoBodyName,
      solver().dt()));
  // clang-format on
  stabilizer_->configure(stabiConfig);

  stabilizer_->setContacts({mc_tasks::lipm_stabilizer::ContactState::Left,
	mc_tasks::lipm_stabilizer::ContactState::Right});

  // Update observers
  datastore().make_call("KinematicAnchorFrame::" + robot().name(), [this](const mc_rbdyn::Robot & robot) {
    return sva::interpolate(robot.surfacePose("RightFootCenter"), robot.surfacePose("LeftFootCenter"), 0.5);
  });

  mc_rtc::log::success("HandReacher init done");
}

bool HandReacher::run()
{
  return mc_control::fsm::Controller::run();
}

void HandReacher::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);

  stabilizer_->reset();
}


CONTROLLER_CONSTRUCTOR("HandReacher", HandReacher)
