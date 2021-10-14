#include "ROSClient.h"

void ROSClient::configure(const mc_rtc::Configuration & config)
{
  config("CtrlServiceName", ctrl_service_);
}

void ROSClient::start(mc_control::fsm::Controller & ctl)
{
  spin_rate_ = (1/ctl.timeStep);

  nh_ = mc_rtc::ROSBridge::get_node_handle();
  client_ = nh_->serviceClient<ros_ml_controller::ControlStep>(ctrl_service_, this);
  spinThread_ = std::thread(std::bind(&ROSClient::rosSpinner, this));

  for (unsigned int i=0; i<rarm_motors.size(); i++){
    int idx = ctl.robot().jointIndexByName(rarm_motors[i]);
    rarm_mbc_ids.push_back(idx);
  }

  createGUI(ctl);
  mc_rtc::log::info("[{}] Node handle created", name());
  mc_rtc::log::info("[{}] Service name: {}", name(), ctrl_service_);

}

bool ROSClient::run(mc_control::fsm::Controller & ctl)
{
  std::vector<double> observation = get_full_state(ctl);

  ros_ml_controller::ControlStep srv;
  srv.request.observation = observation;

  std::vector<double> req(srv.request.observation);
  std::stringstream ss;
  std::copy(req.begin(), req.end(),std::ostream_iterator<double>(ss,", "));
  ROS_DEBUG("Request sent. %s", ss.str().c_str());


  if (client_.call(srv))
  {
    std::vector<double> resp(srv.response.action);
    std::stringstream ss;
    std::copy(resp.begin(), resp.end(),std::ostream_iterator<double>(ss,", "));

    ROS_DEBUG("{%d} Response received. %s", stepCounter_, ss.str().c_str());

    std::vector<double> target;
    for (unsigned int i=0; i<rarm_motors.size(); i++){
      double t = resp.at(i) + motor_offset.at(i)*PI/180;
      target.push_back(t);
    }

    if (active_)
    {
      for (unsigned int i = 0; i < rarm_motors.size(); i++)
      {
	std::string jn = rarm_motors[i];
	ctl.getPostureTask(ctl.robot().name())->target({{jn, std::vector<double>{target[i]}}});
      }

      stepCounter_++;
    }
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }

  output("OK");
  return true;
}

void ROSClient::teardown(mc_control::fsm::Controller & ctl)
{
  running_ = false;
  spinThread_.join();
}


void ROSClient::rosSpinner()
{
  mc_rtc::log::info("ROS spinner thread created");
  ros::Rate r(spin_rate_);
  while(ros::ok() && running_)
  {
    ros::spinOnce();
    r.sleep();
  }
  mc_rtc::log::info("ROS spinner destroyed");
}


void ROSClient::switch_target(mc_control::fsm::Controller & ctl)
{
  auto pos = ctl.realRobot().bodyPosW(ee_link_name) * ctl.realRobot().posW().inv();

  Eigen::Vector3d current_pos(pos.translation());
  wp_iter++;
  if (wp_iter==wp.end()){
    wp_iter = wp.begin();
  }
}

std::vector<double> ROSClient::get_full_state(mc_control::fsm::Controller & ctl)
{
  // external state
  Eigen::Vector3d ext_state(*wp_iter);

  // motor position at joint level
  std::vector<double> motor_pos;
  for (auto i : rarm_mbc_ids){
    motor_pos.push_back(ctl.realRobot().mbc().q[i][0]);
  }
  Eigen::VectorXd eig_motor_pos = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>
    (motor_pos.data(), motor_pos.size());
  // motor velocity at joint level
  std::vector<double> motor_vel;
  for (auto i : rarm_mbc_ids){
    motor_vel.push_back(ctl.realRobot().mbc().alpha[i][0]);
  }
  Eigen::VectorXd eig_motor_vel = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>
    (motor_vel.data(), motor_vel.size());

  // robot state
  Eigen::VectorXd robot_state(18);
  robot_state << eig_motor_pos, eig_motor_vel;

  Eigen::VectorXd full_state(21);
  full_state << robot_state, ext_state;
  std::vector<double> obs(full_state.data(), full_state.data() + full_state.size());
  return obs;
}

void ROSClient::createGUI(mc_control::fsm::Controller & ctl)
{
  auto & gui = *ctl.gui();
  gui.addElement({"RL", name()},
		 mc_rtc::gui::Label("Active", [this]() { return active_; }),
		 mc_rtc::gui::Checkbox("Activated",
				       [this]()
				       {
					 return active_;
				       },
				       [this]()
				       {
					 active_ = !active_;
				       })
		 );
  gui.addElement({"RL", name()},
		 mc_rtc::gui::Button("Next WP", [this, &ctl]() { switch_target(ctl); }));
}

EXPORT_SINGLE_STATE("ROSClient", ROSClient)
