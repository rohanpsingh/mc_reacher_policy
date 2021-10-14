#include "controller.h"

HandReacher::HandReacher(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)

: mc_control::fsm::Controller(rm, dt, config)
{
  config_.load(config);

  mc_rtc::log::success("HandReacher init done");
}

bool HandReacher::run()
{
  return mc_control::fsm::Controller::run();
}

void HandReacher::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}


CONTROLLER_CONSTRUCTOR("HandReacher", HandReacher)
