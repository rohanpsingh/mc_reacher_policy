#include "PolicyClient.h"

void PolicyClient::configure(const mc_rtc::Configuration & config)
{
  config("motorPGains", motor_kps_);
  config("motorDGains", motor_kds_);
}

void PolicyClient::verifyServoGains(mc_control::fsm::Controller & ctl)
{
  const auto & rjo = ctl.robot().module().ref_joint_order();
  for(unsigned int i = 0; i < rarm_motors.size(); i++)
  {
    auto rjo_it = std::find(rjo.begin(), rjo.end(), rarm_motors[i]);
    if(rjo_it == rjo.end())
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("[{}] Motor name \"{}\" not found in ref_joint_order: {}.",
                                                       name(), rarm_motors[i], rjo);
    }

    bool get_servo_success = false;
    double kp, kd;
    if(ctl.datastore().has(ctl.robot().name() + "::GetPDGainsByName"))
    {
      get_servo_success = ctl.datastore().call<bool, const std::string &, double &, double &>(
          ctl.robot().name() + "::GetPDGainsByName", rarm_motors[i], kp, kd);
    }
    if(!get_servo_success)
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("[{}] Could not get PD gains through datastore call!!!", name());
    }

    auto check_and_throw = [&](const std::string & s, double exp, double act) {
      if(std::abs(exp - act) > 0.1)
      {
        mc_rtc::log::error_and_throw<std::runtime_error>(
            "[{}] Unexpected servo {} gain for joint {}!!! Expected={}. Is={}", name(), s, rarm_motors[i], exp, act);
      }
    };
    check_and_throw("kp", motor_kps_[i], kp);
    check_and_throw("kd", motor_kds_[i], kd);
  }
}

void PolicyClient::start(mc_control::fsm::Controller & ctl)
{
  // load policy here
  try
  {
    // Deserialize the ScriptModule from a file using torch::jit::load()
    module = torch::jit::load(path_to_trained_policy_);
  }
  catch(const c10::Error & e)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] Error loading NN model.");
  }

  for(unsigned int i = 0; i < rarm_motors.size(); i++)
  {
    int idx = ctl.robot().jointIndexByName(rarm_motors[i]);
    rarm_mbc_ids.push_back(idx);
  }

  // verify if expected PD gains are loaded
  verifyServoGains(ctl);

  // zero the inputs and outputs to RL
  policy_inputs = std::vector<double>(obs_vec_len, 0);
  policy_actions = std::vector<double>(act_vec_len, 0);

  addLogEntries(ctl);
  createGUI(ctl);
  mc_rtc::log::success("[{}] Trained model loaded from \"{}\"", name(), path_to_trained_policy_);
}

bool PolicyClient::run(mc_control::fsm::Controller & ctl)
{
  int skipFrames = policy_ts / ctl.timeStep;
  if(iterCounter_ % skipFrames == 0)
  {
    // get robot state
    robot_state = get_robot_state(ctl);

    // get external state
    ext_state = get_ext_state(ctl);

    if(robot_state.size() == 0 || ext_state.size() == 0)
    {
      mc_rtc::log::warning("[{}] Observation vector is empty. Model will not execute.", name());
    }

    // create observation vector
    Eigen::VectorXd full_state(obs_vec_len);
    full_state << robot_state, ext_state;
    std::vector<double> obs(full_state.data(), full_state.data() + full_state.size());

    // create fake observation
    std::vector<double> fake_obs = obs;
    fake_obs[6] = -0.82;
    policy_inputs = fake_obs;

    // Convert observation vector to torch::Tensor
    // (https://discuss.pytorch.org/t/how-to-convert-vector-int-into-c-torch-tensor/66539)
    auto opts = torch::TensorOptions().dtype(torch::kDouble);
    torch::Tensor t = torch::from_blob(policy_inputs.data(), {(int)policy_inputs.size()}, opts).to(torch::kFloat);
    std::vector<torch::jit::IValue> inputs = {t};

    // function to do a forward pass through actor network
    std::function<void()> policy_fwd_ = [&, this]() {
      auto start_t = std::chrono::steady_clock::now();
      module_out = module.forward(inputs).toTensor();
      module_out = module_out.contiguous();
      policy_actions = std::vector<double>(module_out.data_ptr<float>(),
                                           module_out.data_ptr<float>() + module_out.numel());
      auto end_t = std::chrono::steady_clock::now();
      exec_dt = end_t - start_t;
    };

    // Call datastore that will execute both the models
    ctl.datastore().call("ExecuteActorCritic", policy_fwd_);

    // for debug
    if(debug_out_ == true)
    {
      std::stringstream obs_ss;
      std::copy(policy_inputs.begin(), policy_inputs.end(), std::ostream_iterator<double>(obs_ss, ", "));
      mc_rtc::log::info("[{}] {} Robot state: {}", name(), stepCounter_, obs_ss.str().c_str());

      std::stringstream act_ss;
      std::copy(policy_actions.begin(), policy_actions.end(), std::ostream_iterator<double>(act_ss, ", "));
      mc_rtc::log::info("[{}] {} Module predictions: {}", name(), stepCounter_, act_ss.str().c_str());
    }
    stepCounter_++;
  }

  if(active_)
  {
    std::vector<double> target;
    if(policy_actions.size() != act_vec_len)
    {
      mc_rtc::log::error("[{}] Action vector is unexpected size (size={} expected={})!", name(),
                         policy_actions.size(), act_vec_len);
      active_ = false;
      output("OK");
      return true;
    }

    for(unsigned int i = 0; i < rarm_motors.size(); i++)
    {
      double t = policy_actions.at(i) + motor_offset.at(i) * PI / 180;
      target.push_back(t);
    }

    // set the control msg
    unsigned int target_idx_ = 0;
    for(auto i : rarm_motors)
    {
      control_msg[i] = target[target_idx_];
      target_idx_++;
    }
    ctl.datastore().assign(datastoreName_, control_msg);
  }

  iterCounter_++;
  output("OK");
  return true;
}

void PolicyClient::addLogEntries(mc_control::fsm::Controller & ctl)
{
  ctl.logger().addLogEntry("Policy_actions", [this]() { return policy_actions; });
  ctl.logger().addLogEntry("Policy_inputs", [this]() { return policy_inputs; });
  ctl.logger().addLogEntry("Policy_eeError", [this]() { return current_ee_error; });
}

void PolicyClient::teardown(mc_control::fsm::Controller & ctl)
{
  active_ = false;
  ctl.gui()->removeCategory({"RL", name()});
  ctl.logger().removeLogEntry("Policy");
}

void PolicyClient::switch_target(mc_control::fsm::Controller & ctl)
{
  wp_iter++;
  if(wp_iter == wp.end())
  {
    wp_iter = wp.begin();
  }
  current_waypoint = *wp_iter;
}

Eigen::VectorXd PolicyClient::get_robot_state(mc_control::fsm::Controller & ctl)
{
  // motor position at joint level
  std::vector<double> motor_pos;
  for(auto i : rarm_mbc_ids)
  {
    motor_pos.push_back(ctl.realRobot().mbc().q[i][0]);
  }
  Eigen::VectorXd eig_motor_pos = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(motor_pos.data(), motor_pos.size());
  // motor velocity at joint level
  std::vector<double> motor_vel;
  for(auto i : rarm_mbc_ids)
  {
    motor_vel.push_back(ctl.realRobot().mbc().alpha[i][0]);
  }
  Eigen::VectorXd eig_motor_vel = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(motor_vel.data(), motor_vel.size());

  // robot state
  Eigen::VectorXd robot_state(18);
  robot_state << eig_motor_pos, eig_motor_vel;
  return robot_state;
}

Eigen::VectorXd PolicyClient::get_ext_state(mc_control::fsm::Controller & ctl)
{
  return current_waypoint;
}

void PolicyClient::createGUI(mc_control::fsm::Controller & ctl)
{
  std::vector<std::string> bodies;
  for(auto body : ctl.robot().mb().bodies())
  {
    bodies.push_back(body.name());
  }

  auto & gui = *ctl.gui();
  gui.addElement({"RL", name()}, mc_rtc::gui::Label("Active", [this]() { return active_; }),
                 mc_rtc::gui::Checkbox(
                     "Activated", [this]() { return active_; }, [this]() { active_ = !active_; }));
  gui.addElement({"RL", name()},
                 mc_rtc::gui::Checkbox(
                     "Debug Out", [this]() { return debug_out_; }, [this]() { debug_out_ = !debug_out_; }));
  gui.addElement({"RL", name()}, mc_rtc::gui::Button("Next WP", [this, &ctl]() { switch_target(ctl); }));
  gui.addElement({"RL", name()}, mc_rtc::gui::Point3D("Current WP (global)", [this, &ctl]() {
                   auto pos = ctl.realRobot().posW() * current_waypoint;
                   return Eigen::Vector3d(pos.translation());
                 }));
  gui.addElement({"RL", name()}, mc_rtc::gui::ComboInput(
                                     "End-effector name", bodies, [this]() { return ee_link_name; },
                                     [this](const std::string & c) { ee_link_name = c; }));
  gui.addElement({"RL", name()}, mc_rtc::gui::Label("Distance from EE", [this, &ctl]() {
                   auto p1 = ctl.realRobot().posW() * current_waypoint;
                   auto p2 = ctl.realRobot().bodyPosW(ee_link_name);
                   current_ee_error = (p1.translation() - p2.translation()).norm();
                   return current_ee_error;
                 }));
  gui.addElement({"RL", name()}, mc_rtc::gui::NumberSlider(
                                     "Relative waypoint (x)", [this]() { return current_waypoint.x(); },
                                     [this](double v) { current_waypoint.x() = v; }, 0.4, 0.6));
  gui.addElement({"RL", name()}, mc_rtc::gui::NumberSlider(
                                     "Relative waypoint (y)", [this]() { return current_waypoint.y(); },
                                     [this](double v) { current_waypoint.y() = v; }, -0.2, -0.4));
  gui.addElement({"RL", name()}, mc_rtc::gui::NumberSlider(
                                     "Relative waypoint (z)", [this]() { return current_waypoint.z(); },
                                     [this](double v) { current_waypoint.z() = v; }, 0.0, 0.4));
}

EXPORT_SINGLE_STATE("PolicyClient", PolicyClient)
