#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/SurfaceTransformTask.h>

struct StabilizerConfiguration : mc_control::fsm::State
{
    void configure(const mc_rtc::Configuration & config) override;
    void start(mc_control::fsm::Controller & ctl) override;
    bool run(mc_control::fsm::Controller & ctl) override;
    void teardown(mc_control::fsm::Controller & ctl) override;

private:
    /// @{ CONFIG
    std::vector<std::string> key_;
    bool verbose_ = false;
    /// @}
};
