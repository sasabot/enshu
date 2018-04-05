#include "aero_mydemo/mydemo_lib.hh"

namespace aero {
  namespace settings {
    negomo_lib::jumpSettings js0;
    negomo_lib::waitSettings ws0; // OPENONE
    negomo_lib::waitSettings wsf; // OPENONE, atEnd, forwardOnly
    negomo_lib::waitSettings wsn; // OPENONE, preinteraction=false, warnavoid=true
  };
}

using namespace aero;

MyDemoLib::MyDemoLib(ros::NodeHandle _nh, bool _multi) : nh_(_nh) {
  // setup robot

  robot_.reset(new aero::interface::AeroMoveitInterface(_nh));
  // robot_->disableWaitInterpolation();
  lib_.reset(new aero::DevelLib(_nh, robot_));
  rotateKinect_ = _nh.advertise<geometry_msgs::Point>("/kinect_controller/look_at", 10);

  // setup default parameters

  settings::ws0.interaction_flag =
    negomo_enshu::PlannerBridgeRequest::Request::OPENONE;

  settings::wsf.interaction_flag =
    negomo_enshu::PlannerBridgeRequest::Request::OPENONE;
  settings::wsf.at_end = 1;
  settings::wsf.backward_allowed = 0;

  settings::wsn.interaction_flag =
    negomo_enshu::PlannerBridgeRequest::Request::OPENONE;
  settings::wsn.enable_preinteraction = false;
  settings::wsn.warn_avoid = true;

  // setup planner

  planner_.reset(new negomo_lib::NegomoBridge2(_nh, "/negomo/", std::bind(&MyDemoLib::moveToWorkspace, this, std::placeholders::_1, std::placeholders::_2)));
  planner_->initlib(boost::bind(&MyDemoLib::actionCallback, this, _1, _2));

  // integrate hand interaction with shop interaction
  if (_multi) {
    handint_.reset(new negomo_lib::NegomoBridge2(_nh, "/negomohand/", std::bind(&MyDemoLib::moveToWorkspace, this, std::placeholders::_1, std::placeholders::_2), 2, 1, false, true));
    handint_->noPlanning();
  }

  pickPlaceTask =
    {std::make_tuple(0, 0, planner_->initTask, planner_->emptyAction, "init"),
     std::make_tuple(0, 0, planner_->moveToWorkspace, planner_->emptyAction, "move"),
     std::make_tuple(0, 0, planner_->loopStart, planner_->emptyAction, "loop"),
     std::make_tuple(0, 1,
                     std::bind(&MyDemoLib::pickFromContainer, this,
                               std::placeholders::_1, std::placeholders::_2),
                     planner_->emptyAction, "pick"),
     std::make_tuple(1, 0,
                     std::bind(&MyDemoLib::placeToShelf, this,
                               std::placeholders::_1, std::placeholders::_2),
                     planner_->emptyAction, "place"),
     std::make_tuple(0, 0, planner_->loopEnd, planner_->emptyAction, "loop"),
     std::make_tuple(0, 0,
                     std::bind(&MyDemoLib::leaveTask, this,
                               std::placeholders::_1, std::placeholders::_2),
                     planner_->emptyAction, "leave"),
     std::make_tuple(0, 0, planner_->finishTask, planner_->emptyAction, "finish")};

  handlePaymentTask =
    {std::make_tuple(0, 0, planner_->initTask, planner_->emptyAction, "init"),
     std::make_tuple(0, 0,
                     std::bind(&MyDemoLib::handlePayment, this,
                               std::placeholders::_1, std::placeholders::_2),
                     planner_->emptyAction, "pay"),
     std::make_tuple(0, 0, planner_->finishTask, planner_->emptyAction, "finish")};

  // robot get out of the way for human operation, re-run once ready
  waitHome =
    {std::make_tuple(0, 0, planner_->initTask, planner_->emptyAction, "init"),
     std::make_tuple(0, 0,
                     std::bind(&MyDemoLib::inform, this,
                               std::placeholders::_1, std::placeholders::_2),
                     planner_->emptyAction, "inform"),
     std::make_tuple(0, 0, planner_->uiExceptionHandle, planner_->emptyAction, "wait"),
     std::make_tuple(0, 0,
                     std::bind(&MyDemoLib::setNextSet, this,
                               std::placeholders::_1, std::placeholders::_2),
                     planner_->emptyAction, "next"),
     std::make_tuple(0, 0, planner_->emptyAction, planner_->emptyAction, "exit"),
     std::make_tuple(0, 0, planner_->finishTask, planner_->emptyAction, "finish")};

  appendq_ = {0, 2};
  customer_x_ = 1.0;
  customer_y_ = 0.0;

  multi_ = _multi;
  target_publisher_ =
    _nh.advertise<geometry_msgs::Point>("/manipulation/target", 10);
}

MyDemoLib::~MyDemoLib() {
}

void MyDemoLib::run(std::vector<negomo_lib::ActionList> _als,
                    negomo_lib::ActionList *_el, negomo_lib::ActionList *_tl) {
  usleep(1000 * 1000); // wait for nodes to be ready
  robot_->setTrackingMode(false);
  robot_->setPoseVariables(aero::pose::reset_manip);
  robot_->sendModelAngles(5000);
  robot_->waitInterpolation();
  rotateKinect(customer_x_, customer_y_, false); // initiate Kinect direction
  planner_->run(_als, _el, _tl);
}

bool MyDemoLib::actionCallback(negomo_enshu::RobotAction::Request &_req,
                               negomo_enshu::RobotAction::Response &_res) {
  ROS_INFO("looking for action %s", _req.action.c_str());
  actions_[_req.action](_req.target_id);
  return true;
}

void MyDemoLib::rotateKinect(float _x, float _y, bool _lock) {
  geometry_msgs::Point msg;
  msg.x = _x; msg.y = _y;
  msg.z = (_lock ? -1 : 1); // negative value means lock
  rotateKinect_.publish(msg);
  usleep(3000 * 1000); // wait for rotate
}

void MyDemoLib::publishTarget(Eigen::Vector3d _pos) {
  geometry_msgs::Point msg;
  msg.x = _pos.x(); msg.y = _pos.y(); msg.z = _pos.z();
  target_publisher_.publish(msg);
}

void MyDemoLib::iStart(negomo_lib::waitSettings _ws) {
  robot_->setTrackingMode(true);
  if (multi_)
    handint_->iStart(settings::js0, _ws);
  planner_->iStart(settings::js0, _ws);
}

int MyDemoLib::iJoin(int &_inhands, int &_nexttask) {
  int next;
  if (multi_ && handint_->iJoin(_inhands, _nexttask) == -404)
    planner_->setError(true, "interrupted during action");
  next = planner_->iJoin(_inhands, _nexttask);
  robot_->setTrackingMode(false);
  return next;
}

// ------------ moveToWorkspace ------------

int MyDemoLib::moveToWorkspace(int _inhands, int &_nexttask) {
  // get waypoints (for this map, use pre-defined locations to avoid path)
  std::vector<std::string> locations;
  locations.push_back(planner_->getEntities().get<std::string>("workspace"));
  int numWaypoints = planner_->getEntities().get<int>("numWaypoints", 0);
  std::string waypoints = planner_->getEntities().get<std::string>("waypoints", "");

  size_t pos = 0;
  for (size_t i = 1; i < numWaypoints; ++i) {
    auto pos2 = waypoints.find(";", pos);
    locations.push_back(waypoints.substr(pos,pos2));
    pos = pos2 + 1;
  }

  // check if already near final location
  if (robot_->isAt(locations.back(), 0.3))
    // only go for final location
    locations.erase(locations.begin(), locations.end() - 1);

  // go to locations
  int count = 0;
  planner_->getEntities().put("isMoving", true);
  for (auto l = locations.begin(); l != locations.end(); ++l) {
    iStart(settings::ws0);
    robot_->moveTo(*l);
    while (true) {
      while (robot_->isMoving() && ros::ok())
        usleep(200 * 1000);
      if (iJoin(_inhands, _nexttask) != -1) // was interrupted
        return _inhands;
      if (robot_->isMoving()) // interrupted but restarted
        iStart(settings::ws0);
      else
        break;
    }
    if (!robot_->isAt(*l, 0.1) && _nexttask == -1 && !abort)
      if (count == 0) { // retry one more time
        ++count;
        --l;
      } else {
        ROS_WARN("failed to go to %s", l->c_str());
        planner_->setBackTrack("failed to go to " + *l);
        _nexttask = -404; // go to error
        return _inhands;
      }
  }
  planner_->getEntities().put("isMoving", false);

  return _inhands;
}

int MyDemoLib::leaveTask(int _inhands, int &_nexttask) {
  // should not be interrupted
  // should go through waypoints, below simplified implementation
  std::string location = "";
  if (planner_->getEntities().get<bool>("leave", true))
    location = planner_->getEntities().get<std::string>("wsleave", "");
  if (location != "" && !lib_->goTo(location)) {
    planner_->setBackTrack("failed leave");
    _nexttask = -404;
  }
  return _inhands;
}

// ------------ pick and place ------------

int MyDemoLib::pickFromContainer(int _inhands, int &_nexttask) {
  lib_->setFCNModel("container");
  // look at container
  planner_->getEntities().put("isMoving", true);
  iStart(settings::ws0);
  lib_->goTo
    (planner_->getEntities().get<std::string>("container", "d_shelf_container"));
  iJoin(_inhands, _nexttask);
  planner_->getEntities().put("isMoving", false);
  if (_nexttask != -1)
    return _inhands;

  aero::arm arm = aero::arm::rarm;

  Eigen::Vector3d pos;
  bool coffee = false;
  int retry_count = planner_->getEntities().get<int>("retry", 0);
  coffee = lib_->poseAndRecognize("container", "caffelatte", pos, -0.12);
  publishTarget(pos);
  settings::wsf.exec_time_ms = 10 * 1000;
  robot_->setLookAt(pos, false, true);
  iStart(settings::wsf);
  if (coffee) { // pick coffee
    coffee = lib_->pickCoffeeFront(pos, 0.93, arm);
    robot_->setPoseVariables(aero::pose::reset_manip);
    robot_->setNeck(0.0, 0.0, 0.0);
    std::map<aero::joint, double> av;
    robot_->getRobotStateVariables(av);
    robot_->sendModelAngles(lib_->calcPathTime(av, 0.8));
    robot_->waitInterpolation();
    _inhands = lib_->getUsingHandsNum();
  }

  planner_->getEntities().put("retry", 0); // reset count (success or error)
  if (!coffee) { // failed recognition or pick
    ++retry_count;
    if (retry_count == 3) { // fatal error
      planner_->setError(true, "failed coffee!");
      _nexttask = -404;
    } else { // we can retry
      planner_->getEntities().put("retry", retry_count); // update retry count
      planner_->redoThis();
    }
  }
  iJoin(_inhands, _nexttask);

  return lib_->getUsingHandsNum();
}

int MyDemoLib::placeToShelf(int _inhands, int &_nexttask) {
  int remainingItems = planner_->getEntities().get<int>("remain");

  iStart(settings::wsn);
  lib_->lookShelf();
  if (iJoin(_inhands, _nexttask) != -1)
    return _inhands;

  aero::arm arm = aero::arm::rarm;
  float placeto = planner_->getEntities().get<float>("placeto", 0.0);

  iStart(settings::wsn);
  lib_->goTo
    (planner_->getEntities().get<std::string>("shelf", "d_shelf"));
  if (iJoin(_inhands, _nexttask) != -1)
    return _inhands;
  settings::wsf.exec_time_ms = 15 * 1000;
  iStart(settings::wsf);
  lib_->placeCoffee(Eigen::Vector3d(0.75, -0.25, 1.05), placeto, arm);
  robot_->setPoseVariables(aero::pose::reset_manip);
  robot_->setNeck(0.0, 0.0, 0.0);
  std::map<aero::joint, double> av;
  robot_->getRobotStateVariables(av);
  robot_->sendModelAngles(lib_->calcPathTime(av, 0.8));
  robot_->waitInterpolation();
  _inhands = lib_->getUsingHandsNum();
  iJoin(_inhands, _nexttask);

  // update remain count
  planner_->getEntities().put("remain", remainingItems - 1);
  planner_->getEntities().put("placeto", placeto + 0.1);
  if (remainingItems <= 1) // now remaining = 0
    planner_->getEntities().put("loopCondition", false);

  lib_->lookShelf();
  return lib_->getUsingHandsNum();
}

// ------------ payment ------------

int MyDemoLib::handlePayment(int _inhands, int &_nexttask) {
  // face toward shelf for interaction
  std::string shelf_name =
    planner_->getEntities().get<std::string>("shelf", "s_shelf");
  lib_->goTo(shelf_name);
  usleep(1000 * 1000);
  // main interaction
  lookAtCustomer();
  recognizeUserItemWithGesture();
  // on exit
  rotateKinect(customer_x_, customer_y_, false);
  robot_->setTrackingMode(false);
  usleep(1000 * 1000);
  planner_->iOnTimered(10000); // no interaction for next 10 seconds
  return _inhands;
}

// ------------ waitHome ------------

int MyDemoLib::inform(int _inhands, int &_nexttask) {
  lib_->speak("Task complete, entering wait mode.", 3.5);
  robot_->setPoseVariables(aero::pose::reset_manip);
  robot_->sendModelAngles(5000);
  robot_->waitInterpolation();
  return _inhands;
}

int MyDemoLib::setNextSet(int _inhands, int &_nexttask) {
  planner_->appendDefaultQueue(appendq_);
  planner_->goToEnd(); // skip exit and continue
  return _inhands;
}
