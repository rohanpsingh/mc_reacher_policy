#pragma once

#include <torch/torch.h>
#include <torch/script.h>

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>
#include <mc_tasks/PostureTask.h>
#include <mc_rtc/logging.h>

#define PI 3.14159265

struct PolicyClient : mc_control::fsm::State
{
    void configure(const mc_rtc::Configuration & config) override;
    void start(mc_control::fsm::Controller & ctl) override;
    bool run(mc_control::fsm::Controller & ctl) override;
    void teardown(mc_control::fsm::Controller & ctl) override;

protected:
    void createGUI(mc_control::fsm::Controller & ctl);
    Eigen::VectorXd get_robot_state(mc_control::fsm::Controller & ctl);
    Eigen::VectorXd get_ext_state(mc_control::fsm::Controller & ctl);
    void switch_target(mc_control::fsm::Controller & ctl);

private:
    // trained policy
    const std::string path_to_trained_policy_ = "/tmp/actor.pt";
    const int base_obs_len = 21;
    torch::jit::script::Module module;

    // Set 'true' to enable PostureTask targets update
    bool active_ = false;

    // Set 'true' to print debug info
    bool debug_out_ = false;

    // List of activated motors
    const std::vector<std::string> rarm_motors = {"RSC", "RSP", "RSR", "RSY", "REP",
						  "RWRY", "RWRR", "RWRP", "RHDY"};
    std::vector<int> rarm_mbc_ids;

    // Counter for forward passes
    int stepCounter_ = 0;

    // Name of end-effector to display distance in GUI
    std::string ee_link_name = "Rindex_Link2";

    // Fixed offsets to be added to NN predictions
    const std::vector<double> motor_offset = {0, 60, -20,  -5, -105,  0, -40, 0, 0,
					      0, 60,  20,   5, -105,  0,  40, 0, 0};

    // Fixed waypoints
    const std::vector<Eigen::Vector3d> wp = {Eigen::Vector3d(0.5, -0.2, 0.0),
					     Eigen::Vector3d(0.5, -0.2, 0.4),
					     Eigen::Vector3d(0.5, -0.4, 0.4),
					     Eigen::Vector3d(0.5, -0.4, 0.0)};
    std::vector<Eigen::Vector3d>::const_iterator wp_iter = wp.begin();
};
