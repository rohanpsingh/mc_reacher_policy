#include "controller.h"

HandReacher::HandReacher(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)

: mc_control::fsm::Controller(rm, dt, config)
{
  config_.load(config);

  // create datastore for getting policy predictions
  datastore().make<std::map<std::string, double>>(datastoreName_);

  // create new thread for executing the policies
  policy_th_ = std::thread([this]() {
    while(!policy_th_exit_)
    {
      std::unique_lock lk(policy_mutex_);
      cv.wait(lk, [this] { return (policy_work_ != nullptr); });

      (*policy_work_)();
      policy_work_ = nullptr;

      lk.unlock();
      cv.notify_one();
    }
  });

  // create gui
  gui()->addElement({"RL"}, mc_rtc::gui::Label("Policy thread id:", [this]() {
                      std::stringstream ss;
                      ss << policy_th_.get_id();
                      return ss.str();
                    }));

  // Load default configuration from robot module
  mc_rtc::log::info("Loading default stabilizer configuration");
  auto stabiConfig = robot().module().defaultLIPMStabilizerConfiguration();
  auto robotConfig = config("robot_models")(controlRobot().name());
  if(robotConfig.has("stabilizer"))
  {
    mc_rtc::log::info("Loading additional stabilizer configuration:\n{}", robotConfig("stabilizer").dump(true));
    stabiConfig.load(robotConfig("stabilizer"));
    mc_rtc::log::info("Stabi prop {}", stabiConfig.dcmPropGain);
  }

  // configure stabilizer task
  stabilizer_.reset(new mc_tasks::lipm_stabilizer::StabilizerTask(
      solver().robots(), solver().realRobots(), robot().robotIndex(), stabiConfig.leftFootSurface,
      stabiConfig.rightFootSurface, stabiConfig.torsoBodyName, solver().dt()));
  stabilizer_->configure(stabiConfig);

  // is this needed?
  stabilizer_->setContacts(
      {mc_tasks::lipm_stabilizer::ContactState::Left, mc_tasks::lipm_stabilizer::ContactState::Right});

  // Update observers
  datastore().make_call("KinematicAnchorFrame::" + robot().name(), [this](const mc_rbdyn::Robot & robot) {
    return sva::interpolate(robot.surfacePose("RightFootCenter"), robot.surfacePose("LeftFootCenter"), 0.5);
  });

  mc_rtc::log::success("HandReacher init done");
}

HandReacher::~HandReacher()
{
  std::function<void()> exit = []() {};

  {
    std::lock_guard lk(policy_mutex_);
    policy_th_exit_ = true;
    policy_work_ = &exit;
  }
  cv.notify_one();
  policy_th_.join();
}

bool HandReacher::run()
{
  if(mc_control::fsm::Controller::run())
  {
    if(!datastore().has(datastoreName_))
    {
      return true;
    }
    msg = datastore().get<std::map<std::string, double>>(datastoreName_);
    for(auto i : msg)
    {
      auto jIndex = robot().jointIndexByName(i.first);
      robot().mbc().q[jIndex] = std::vector<double>{i.second};
      robot().mbc().alpha[jIndex] = std::vector<double>{0};
    }
    return true;
  }
  return false;
}

void HandReacher::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);

  stabilizer_->reset();

  // create datastore that can be called by FSM state
  datastore().make_call("ExecuteActorCritic", [this](std::function<void()> & work) {
    {
      std::lock_guard lk(policy_mutex_);
      policy_work_ = &work;
    }
    cv.notify_one();

    {
      std::unique_lock lk(policy_mutex_);
      cv.wait(lk, [this] { return (policy_work_ == nullptr); });
    }
  });
}
