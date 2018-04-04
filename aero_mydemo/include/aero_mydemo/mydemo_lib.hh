#ifndef AERO_MYDEMO_LIB_
#define AERO_MYDEMO_LIB_

#include "negomo/NegomoLib2.hh"
#include "aero_devel_lib/devel_lib.hh"
#include <roboenvcv/interaction.h>
#include <roboenvcv/PersonCoordinate.h>

namespace aero {

  class MyDemoLib {

  public: explicit MyDemoLib(ros::NodeHandle _nh, bool _multi=false);

  public: ~MyDemoLib();

  public: void run(std::vector<negomo_lib::ActionList> _als,
                   negomo_lib::ActionList *_el, negomo_lib::ActionList *_tl);

  // preinteraction callback
  private: bool actionCallback(negomo_enshu::RobotAction::Request &_req,
                               negomo_enshu::RobotAction::Response &_res);

  // utils

  public: void rotateKinect(float _x, float _y, bool _lock);

  public: void publishTarget(Eigen::Vector3d _pos);

  public: void iStart(negomo_lib::waitSettings _ws);

  public: int iJoin(int &_inhands, int &_nexttask);

  // action nodes

  private: int goTo(int _inhands, int &_nexttask,
                    std::vector<std::string> locations,
                    bool abort=false, int numWaypoints=0,
                    std::string waypoints="");

  public: int moveToWorkspace(int _inhands, int &_nexttask);

  public: int leaveTask(int _inhands, int &_nexttask);

  public: int pickFromContainer(int _inhands, int &_nexttask);

  public: int placeToShelf(int _inhands, int &_nexttask);

  public: int handlePayment(int _inhands, int &_nexttask);

  public: int inform(int _inhands, int &_nexttask);

  public: int setNextSet(int _inhands, int &_nexttask);

  public: int pickInContainer(int _inhands, int &_nexttask);

  public: int placeInContainer(int _inhands, int &_nexttask);

  // implement here action function

  // other functions in handlePayment.cc

  private: struct regitem {
    int id;
    std::string name;
    int price;
  };

  private: struct regitemrec {
    int count;
    ros::Time t;
    bool found;
  };

  private: static const std::map<std::string, regitem> regitems;

  private: void lookAtCustomer();

  private: void recognizeUserItemWithGesture();

  private: void gestureCallback_(const roboenvcv::PersonCoordinatePtr _msg);

  // variables

  private: ros::NodeHandle nh_;

  public: negomo_lib::NegomoBridge2Ptr planner_;

  public: negomo_lib::NegomoBridge2Ptr handint_;

  public: aero::interface::AeroMoveitInterface::Ptr robot_;

  public: aero::DevelLibPtr lib_;

  private: ros::Publisher rotateKinect_;

  private: std::vector<int> appendq_;

  public: std::map<std::string, std::function<void(int)> > actions_;

  public: float customer_x_, customer_y_;

  private: bool multi_;

  private: roboenvcv::PersonCoordinate gesture_msg_;

  private: ros::Publisher target_publisher_;

  public: negomo_lib::ActionList pickPlaceTask;

  public: negomo_lib::ActionList handlePaymentTask;

  public: negomo_lib::ActionList waitHome;

  public: negomo_lib::ActionList moveItemTask;

  // implement here action list of task
  };

  typedef std::shared_ptr<MyDemoLib> MyDemoLibPtr;

};

#endif
