#pragma once

#include <mc_rtc/ros.h>
#include <ros/ros.h>
#include <ros_ml_controller/ControlStep.h>
#include <thread>
#include <math.h>

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>
#include <mc_tasks/PostureTask.h>
#include <mc_rtc/logging.h>

#define PI 3.14159265

struct ROSClient : mc_control::fsm::State
{
    void configure(const mc_rtc::Configuration & config) override;
    void start(mc_control::fsm::Controller & ctl) override;
    bool run(mc_control::fsm::Controller & ctl) override;
    void teardown(mc_control::fsm::Controller & ctl) override;

protected:
    void createGUI(mc_control::fsm::Controller & ctl);
    void rosSpinner();
    void switch_target(mc_control::fsm::Controller & ctl);
    std::vector<double> get_full_state(mc_control::fsm::Controller & ctl);

private:
    std::string ctrl_service_ = "/rl_server/step_nn";

    /**
     * Handing the ROS thread
     * @{
     */
    std::shared_ptr<ros::NodeHandle> nh_;
    ros::ServiceClient client_;
    std::thread spinThread_;
    int spin_rate_ = 0;

    bool running_ = true;
    bool active_ = false;
    const std::vector<std::string> rarm_motors = {"RSC", "RSP", "RSR", "RSY", "REP",
						  "RWRY", "RWRR", "RWRP", "RHDY"};
    std::vector<int> rarm_mbc_ids;
    int stepCounter_ = 0;

    const std::vector<double> motor_offset = {0, 60, -20,  -5, -105,  0, -40, 0, 0,
					      0, 60,  20,   5, -105,  0,  40, 0, 0};

    const std::string ee_link_name = "Rindex_Link2";
    const std::vector<Eigen::Vector3d> wp = {Eigen::Vector3d(0.5, -0.2, 0.0),
					     Eigen::Vector3d(0.5, -0.2, 0.4),
					     Eigen::Vector3d(0.5, -0.4, 0.4),
					     Eigen::Vector3d(0.5, -0.4, 0.0)};
    std::vector<Eigen::Vector3d>::const_iterator wp_iter = wp.begin();
    /// @}
};
