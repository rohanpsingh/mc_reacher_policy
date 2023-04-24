#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>
#include <mc_rtc/logging.h>

struct ModifyGains : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

protected:
  void modifyServoGains(mc_control::fsm::Controller & ctl,
                        const std::vector<double> & kp,
                        const std::vector<double> & kd);
  void createGUI(mc_control::fsm::Controller & ctl);
  void addLogEntries(mc_control::fsm::Controller & ctl);

private:
  /// @{ CONFIG
  /**
   * Active motor names should be provided in the same order as policy predictions.
   */
  std::vector<std::string> active_motors_;
  /**
   * Servo kP gains should be provided in the same order as active_motors_.
   */
  std::vector<double> target_kps_;
  /**
   * Servo kD gains should be provided in the same order as active_motors_.
   */
  std::vector<double> target_kds_;
  /**
   * Time duration (s) over which to interpolate and modify PD gains.
   */
  double duration_ = 5;
  ///@}

  // Holder for initial PD gains (update at Start of this state)
  std::vector<double> start_kps_;
  std::vector<double> start_kds_;

  // Counter for run iterations
  int iterCounter_ = 0;
};
