#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_control/mc_controller.h>
#include <mc_tasks/lipm_stabilizer/StabilizerTask.h>

#include "api.h"

struct HandReacher_DLLAPI HandReacher : public mc_control::fsm::Controller
{
  HandReacher(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  mc_rbdyn::Robot & controlRobot()
  {
    return mc_control::fsm::Controller::robot();
  }

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

private:
  mc_rtc::Configuration config_;

  std::shared_ptr<mc_tasks::lipm_stabilizer::StabilizerTask> stabilizer_;
  mc_rbdyn::lipm_stabilizer::StabilizerConfiguration
      defaultStabilizerConfig_; /**< Default configuration of the stabilizer */
};
