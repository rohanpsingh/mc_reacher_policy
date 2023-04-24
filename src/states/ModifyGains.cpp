#include "ModifyGains.h"

void ModifyGains::configure(const mc_rtc::Configuration & config)
{
  config("activeMotors", active_motors_);
  config("motorPGains", target_kps_);
  config("motorDGains", target_kds_);
  config("duration", duration_);
}

void ModifyGains::modifyServoGains(mc_control::fsm::Controller & ctl,
                                   const std::vector<double> & kp,
                                   const std::vector<double> & kd)
{
  bool set_servo_success = false;
  {
    if(ctl.datastore().has(ctl.robot().name() + "::SetPDGains"))
    {
      set_servo_success = ctl.datastore().call<bool, const std::vector<double> &, const std::vector<double> &>(
          ctl.robot().name() + "::SetPDGains", kp, kd);
    }

    if(!set_servo_success)
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("[{}] Could not set PD gains through datastore call!!!", name());
    }
  }
}

void ModifyGains::start(mc_control::fsm::Controller & ctl)
{
  // read default pd gains
  bool get_servo_success = false;
  if(ctl.datastore().has(ctl.robot().name() + "::GetPDGains"))
  {
    get_servo_success = ctl.datastore().call<bool, std::vector<double> &, std::vector<double> &>(
        ctl.robot().name() + "::GetPDGains", start_kps_, start_kds_);
  }

  if(!get_servo_success)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] Could not get PD gains through datastore call!!!", name());
  }

  // sanity check
  const auto & rjo = ctl.robot().module().ref_joint_order();
  for(unsigned int i = 0; i < active_motors_.size(); i++)
  {
    auto rjo_it = std::find(rjo.begin(), rjo.end(), active_motors_[i]);
    if(rjo_it != rjo.end())
    {
      mc_rtc::log::info("[{}] Joint=\"{}\". Will modify kp: {} -> {} and kd: {} -> {}.", name(), active_motors_[i],
                        start_kps_[i], target_kps_[i], start_kds_[i], target_kds_[i]);
    }
    else
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("[{}] Motor name \"{}\" not found in ref_joint_order.", name(),
                                                       active_motors_[i]);
    }
  }

  addLogEntries(ctl);
  createGUI(ctl);
  mc_rtc::log::success("[{}] Starting servo PD gain modification.", name());
}

bool ModifyGains::run(mc_control::fsm::Controller & ctl)
{
  const int n = duration_ / ctl.timeStep;
  if(iterCounter_ >= n)
  {
    output("OK");
    return true;
  }

  // set new pd gains
  std::vector<double> kp = start_kps_;
  std::vector<double> kd = start_kds_;
  const auto & rjo = ctl.robot().module().ref_joint_order();
  for(unsigned int i = 0; i < active_motors_.size(); i++)
  {
    auto rjo_it = std::find(rjo.begin(), rjo.end(), active_motors_[i]);
    if(rjo_it != rjo.end())
    {
      int rjo_idx = std::distance(rjo.begin(), rjo_it);
      kp[rjo_idx] = start_kps_[rjo_idx] + (iterCounter_ + 1) * (target_kps_[i] - start_kps_[rjo_idx]) / n;
      kd[rjo_idx] = start_kds_[rjo_idx] + (iterCounter_ + 1) * (target_kds_[i] - start_kds_[rjo_idx]) / n;
    }
  }
  modifyServoGains(ctl, kp, kd);

  iterCounter_++;
  output("OK");
  return false;
}

void ModifyGains::addLogEntries(mc_control::fsm::Controller & ctl) {}

void ModifyGains::teardown(mc_control::fsm::Controller & ctl)
{
  ctl.gui()->removeCategory({"RL", name()});
}

void ModifyGains::createGUI(mc_control::fsm::Controller & ctl)
{
  auto & gui = *ctl.gui();
}

EXPORT_SINGLE_STATE("ModifyGains", ModifyGains)
