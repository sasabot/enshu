#include "aero_mydemo/mydemo_lib.hh"

aero::MyDemoLibPtr an_;


// ------------ preinteraction action nodes ------------

void proactive0(int _target) {}; // not used in this demo
void proactive1(int _target) {}; // not used in this demo
void reactiveT0(int _target) {
  an_->planner_->getEntities().put("target", _target);
  if (an_->robot_->isMoving()) // if robot is moving stop
    an_->robot_->stop();
  auto head = an_->planner_->getHeadPos(_target);
  an_->robot_->setLookAt(std::get<0>(head), std::get<1>(head), std::get<2>(head), true, true, false);
  usleep(1000 * 1000);
};
void returnPlanner(int _target) {
  an_->robot_->setLookAtTopic("/look_at/previous");
  if (an_->planner_->getEntities().get<bool>("isMoving", false)) {
    an_->robot_->go();
    while (an_->robot_->isMoving() && ros::ok())
      usleep(200 * 1000);
    usleep(1000 * 1000);
  }
  usleep(1000 * 1000);
};

// ------------ exception action nodes ------------

int getHelp(int _inhands, int& _nexttask) {
  an_->lib_->speak("I have detected error! " + an_->planner_->getBackTrack() + " I call for human help.", 8.3);
  return _inhands;
}

// ------------ main ------------

int main(int argc, char **argv) {
  ros::init(argc, argv, "shop_sample");
  ros::NodeHandle nh;

  // setup robot

  an_.reset(new aero::MyDemoLib(nh));

  // get parameters

  std::string shelf_name = "s_shelf";
  nh.getParam("customer_x", an_->customer_x_);
  nh.getParam("customer_y", an_->customer_y_);
  nh.getParam("shelf_name", shelf_name);

  // setup negomo

  an_->actions_["/negomo/reset"] = returnPlanner;
  an_->actions_["/negomo/proactive0"] = proactive0; // not used in this demo
  an_->actions_["/negomo/proactive1"] = proactive1; // not used in this demo
  an_->actions_["/negomo/reactive0"] = reactiveT0;

  // setup planner

  negomo_lib::ActionList exceptions =
    {std::make_tuple(0, 0, getHelp, an_->planner_->emptyAction, "getHelp"),
     std::make_tuple(0, 0, an_->planner_->uiExceptionHandle, an_->planner_->emptyAction, "onHelp")};

  negomo_lib::ActionList temps = // not used in this demonstration
    {std::make_tuple(2, 0, an_->planner_->emptyAction, an_->planner_->emptyAction, "tmp")};

  // create task capabilities

  std::vector<negomo_lib::ActionList> tasks_shop =
    {an_->pickPlaceTask, an_->handlePaymentTask, an_->waitHome};
  std::vector<std::vector<std::string> > entities_shop =
    {{"pickPlaceTask","workspace="+shelf_name+"_enter","wsleave="+shelf_name+"_leave","numWaypoints=2","waypoints="+shelf_name+"_container","shelf="+shelf_name,"container="+shelf_name+"_container",
      "remain=2","leave=false","loopCondition=true","iOff=false"},
     {"handlePaymentTask","priority=0","person=anonymous","workspace=HERE","shelf="+shelf_name},
     {"waitHome","workspace=HERE","blocked=true","iOff=true"}};

  an_->planner_->init(entities_shop);
  std::vector<int> defaultq = {0, 2};
  an_->planner_->setDefaultQueue(defaultq);

  an_->run(tasks_shop, &exceptions, &temps);
}
