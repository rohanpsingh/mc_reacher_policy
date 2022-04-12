#include "PolicyClient.h"

void PolicyClient::configure(const mc_rtc::Configuration & config)
{
}

void PolicyClient::start(mc_control::fsm::Controller & ctl)
{
  // load policy here
  try {
    // Deserialize the ScriptModule from a file using torch::jit::load()
    module = torch::jit::load(path_to_trained_policy_);
  }
  catch (const c10::Error& e) {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] Error loading NN model.");
  }

  for (unsigned int i=0; i<rarm_motors.size(); i++){
    int idx = ctl.robot().jointIndexByName(rarm_motors[i]);
    rarm_mbc_ids.push_back(idx);
  }

  createGUI(ctl);
  mc_rtc::log::success("[{}] Trained model loaded from \"{}\"", name(), path_to_trained_policy_);
}

bool PolicyClient::run(mc_control::fsm::Controller & ctl)
{
  // get robot state
  Eigen::VectorXd robot_state = get_robot_state(ctl);
  // get external state
  Eigen::VectorXd ext_state = get_ext_state(ctl);

  // create observation vector
  Eigen::VectorXd full_state(base_obs_len);
  full_state << robot_state, ext_state;
  std::vector<double> obs(full_state.data(), full_state.data() + full_state.size());

  /*
  // for debug
  std::stringstream ss;
  std::copy(obs.begin(), obs.end(),std::ostream_iterator<double>(ss,", "));
  mc_rtc::log::info("[{}] {%d} Robot state: %s", name(), stepCounter_, ss.str().c_str());
  */

  if (obs.size())
  {
    // forward pass through policy
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(torch::ones(base_obs_len));

    // Execute the model and turn its output into a tensor.
    at::Tensor module_out = module.forward(inputs).toTensor();
    std::vector<double> preds(module_out.data_ptr<double>(),
			      module_out.data_ptr<double>() + module_out.numel());

    /*
    // for debug
    std::stringstream ss;
    std::copy(preds.begin(), preds.end(),std::ostream_iterator<double>(ss,", "));
    mc_rtc::log::info("[{}] {%d} Module predictions: %s", stepCounter_, ss.str().c_str());
    */

    // add fixed motor offsets to the predictions
    std::vector<double> target;
    for (unsigned int i=0; i<rarm_motors.size(); i++){
      double t = preds.at(i) + motor_offset.at(i)*PI/180;
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
    mc_rtc::log::error("Observation vector is empty");
    return 1;
  }

  output("OK");
  return true;
}

void PolicyClient::teardown(mc_control::fsm::Controller & ctl)
{
  active_ = false;
  ctl.gui()->removeCategory({"RL", name()});
}


void PolicyClient::switch_target(mc_control::fsm::Controller & ctl)
{
  auto pos = ctl.realRobot().bodyPosW(ee_link_name) * ctl.realRobot().posW().inv();

  Eigen::Vector3d current_pos(pos.translation());
  wp_iter++;
  if (wp_iter==wp.end()){
    wp_iter = wp.begin();
  }
}

Eigen::VectorXd PolicyClient::get_robot_state(mc_control::fsm::Controller & ctl)
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
  return robot_state;
}

Eigen::VectorXd PolicyClient::get_ext_state(mc_control::fsm::Controller & ctl)
{
  // external state
  Eigen::Vector3d ext_state(*wp_iter);
  return ext_state;
}

void PolicyClient::createGUI(mc_control::fsm::Controller & ctl)
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

EXPORT_SINGLE_STATE("PolicyClient", PolicyClient)
