#include "StabilizerConfiguration.h"

#include <mc_control/fsm/Controller.h>
#include <lipm_walking/Controller.h>
#include <mc_rtc/io_utils.h>

void StabilizerConfiguration::configure(const mc_rtc::Configuration & config)
{
  config("key", key_);
  config("verbose", verbose_);
}

void StabilizerConfiguration::start(mc_control::fsm::Controller & ctl)
{
  if(key_.empty())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] You must specify a \"key\" entry in the configuration", name());
  }
  if(!ctl.datastore().has("StabilizerStandingState::getConfiguration"))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] The StabilizerStandingState must be running before this state", name());
  }

  auto config = ctl.datastore().call<mc_rbdyn::lipm_stabilizer::StabilizerConfiguration>("StabilizerStandingState::getConfiguration");
  auto newConf = ctl.config()(key_.front());
  for(auto it = std::next(std::begin(key_)); it != std::end(key_); ++it)
  {
    newConf = newConf(*it);
  }
  config.load(newConf);
  mc_rtc::log::info("[{}] Changing stabilizer configuration", name());
  if(verbose_)
  {
    mc_rtc::log::info("Entries changed:\n{}", newConf.dump(true));
  }
  ctl.datastore().call<void, const mc_rbdyn::lipm_stabilizer::StabilizerConfiguration &>("StabilizerStandingState::setConfiguration", config);
}

bool StabilizerConfiguration::run(mc_control::fsm::Controller & ctl_)
{

  output("OK");
  return true;
}

void StabilizerConfiguration::teardown(mc_control::fsm::Controller & ctl)
{
}

EXPORT_SINGLE_STATE("StabilizerConfiguration", StabilizerConfiguration)
