#pragma once

#include <torch/script.h>
#include <torch/torch.h>

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>
#include <mc_rtc/logging.h>
#include <mc_tasks/PostureTask.h>

#define PI 3.14159265

struct PolicyClient : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;

protected:
  void verifyServoGains(mc_control::fsm::Controller & ctl);
  void triggerSoftEmergency(mc_control::fsm::Controller & ctl);
  void createGUI(mc_control::fsm::Controller & ctl);
  Eigen::VectorXd get_robot_state(mc_control::fsm::Controller & ctl);
  Eigen::VectorXd get_ext_state(mc_control::fsm::Controller & ctl);
  void switch_target(mc_control::fsm::Controller & ctl);
  void addLogEntries(mc_control::fsm::Controller & ctl);

private:
  /// @{ CONFIG
  /**
   * Servo kP gains should be provided in the same order as active_motors_.
   */
  std::vector<double> motor_kps_;
  /**
   * Servo kD gains should be provided in the same order as active_motors_.
   */
  std::vector<double> motor_kds_;
  /**
   * Speed limits for joints. Should be in same order as active_motors_.
   */
  std::vector<double> qdot_limits_;
  /**
   * Control torque limits for joints. Should be in same order as active_motors_.
   */
  std::vector<double> tau_limits_;
  /**
   * Lower limit (in degrees) for joint position limits.
   */
  std::vector<double> qlim_lower_;
  /**
   * Upper limit (in degrees) for joint position limits.
   */
  std::vector<double> qlim_upper_;
  ///@}

  // control msg to send to actuators
  std::map<std::string, double> control_msg;
  // name of datastore that carries control msg to controller
  std::string datastoreName_ = "PolicyPredictions";

  // trained policy
  const std::string path_to_trained_policy_ = "/tmp/actor.pt";
  const int obs_vec_len = 21;
  const int act_vec_len = 9;
  const float policy_ts = 0.025;
  torch::jit::script::Module module;

  // Holder for observations(robot_state + external state) and actions
  std::vector<double> policy_inputs;
  std::vector<double> policy_actions;
  Eigen::VectorXd robot_state;
  Eigen::VectorXd ext_state;

  // Torque commanded by policy
  std::vector<double> command_torques;

  // Set 'true' to enable PostureTask targets update
  bool active_ = false;

  // Set 'true' to print debug info
  bool debug_out_ = false;

  // Set 'true' to trigger emergency
  bool eme_raised = false;

  // Holders for NN raw outputs
  at::Tensor module_out;

  // List of activated motors
  std::vector<std::string> rarm_motors = {"RSC", "RSP", "RSR", "RSY", "REP", "RWRY", "RWRR", "RWRP", "RHDY"};
  std::vector<int> rarm_mbc_ids;

  // Counter for forward passes
  int stepCounter_ = 0;
  // Counter for run iterations
  int iterCounter_ = 0;

  // Name of end-effector to display distance in GUI
  std::string ee_link_name = "Rhand_Link0_Plan2";

  // Fixed offsets to be added to NN predictions (degrees)
  // ideally, we should get this from half-sitting posture
  const std::vector<double> motor_offset = {0, 60, -20, -5, -105, 0, -40, 0, 0};

  // Fixed waypoints
  const std::vector<Eigen::Vector3d> wp = {Eigen::Vector3d(0.5, -0.2, 0.0), Eigen::Vector3d(0.5, -0.2, 0.4),
                                           Eigen::Vector3d(0.5, -0.4, 0.4), Eigen::Vector3d(0.5, -0.4, 0.0)};
  std::vector<Eigen::Vector3d>::const_iterator wp_iter = wp.begin();
  Eigen::Vector3d current_waypoint = Eigen::Vector3d(0.4, -0.3, 0.1);
  float current_ee_error = 0.0;

  std::chrono::duration<double, std::milli> exec_dt{0};
};
