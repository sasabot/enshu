#include "aero_mydemo/mydemo_lib.hh"

using namespace aero;

void MyDemoLib::lookAtCustomer() {
  // get now pose
  robot_->setPoseVariables(aero::pose::reset_manip);
  // convert map coordinates to base coordinates
  Eigen::Vector3d direction_base =
    robot_->volatileTransformToBase(1.6, 0.1, 1.6);
  // get joint limit
  auto bounds =
    robot_->kinematic_model->getVariableBounds
    (aero::joint_map.at(aero::joint::waist_y));
  // set joint angle pose
  double waist_angle = atan2(direction_base.y(), direction_base.x());
  if (waist_angle < 0)
    waist_angle = std::max(waist_angle, bounds.min_position_);
  else
    waist_angle = std::min(waist_angle, bounds.max_position_);
  robot_->setJoint(aero::joint::waist_y, waist_angle);
  // set Kinect slightly diagnal to get looking region
  rotateKinect(0.966, -0.259, false);
  // look at given position
  Eigen::Vector3d recalc_direction_base = // torso moved
    robot_->volatileTransformToBase(1.6, 0.1, 1.6);
  robot_->setLookAt(recalc_direction_base);
  robot_->setLifter(0.0, -0.1);
  robot_->sendModelAngles(3000);
  robot_->waitInterpolation();
}

const std::map<std::string, MyDemoLib::regitem> MyDemoLib::regitems = {
      {"kinoko", {1, "きのこの山", 198}},
      {"takenoko", {1, "たけのこのさと", 198}},
      {"macadamia", {2, "マカダミア", 210}},
      {"koara", {3, "こあらのまーち", 108}},
      {"pie", {4, "パイの実", 108}},
      {"caffelatte", {5, "かふぇらて", 108}},
      {"milktea", {6, "みるくてぃー", 108}},
      {"mixjuice", {7, "みっくすじゅーす", 108}}
};

void MyDemoLib::recognizeUserItemWithGesture() {
  lib_->speakAsync("承ります");
  int look_count = 0;
  bool already_demonstrated = true; // false
  int total_price = 0;
  int finish_count = 0;
  std::map<std::string, regitemrec> recognizedReg;
  for (auto &it: regitems)
    recognizedReg[it.first] = {0, ros::Time::now(), false};
  lib_->startFCN();
  lib_->setObject3DProjectorMode(0);
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::StrParameter msg;
  msg.name = "model_name";
  msg.value = "final";
  srv_req.config.strs = {msg};
  ros::service::call("/object_detector/set_parameters", srv_req, srv_resp);
  ros::Time now = ros::Time::now();
  ros::Subscriber gesture_sub_ =
    nh_.subscribe("/roboenvcv/personcoordinate/global/withid", 1, &MyDemoLib::gestureCallback_, this);
  // get recognition results 
  while (true) {
    usleep(100 * 1000);
    ros::spinOnce();    
    // check for gesture
    roboenvcv::PersonCameraCoords person;
    person.position3d =
      Eigen::Vector3f(gesture_msg_.position3d_camera.x,
                      gesture_msg_.position3d_camera.y,
                      gesture_msg_.position3d_camera.z);
    person.p_base_to_camera =
      Eigen::Vector3f(gesture_msg_.map_to_camera.position.x,
                      gesture_msg_.map_to_camera.position.y,
                      gesture_msg_.map_to_camera.position.z);
    person.mat_base_to_camera =
      Eigen::Quaternionf(gesture_msg_.map_to_camera.orientation.x,
                         gesture_msg_.map_to_camera.orientation.y,
                         gesture_msg_.map_to_camera.orientation.z,
                         gesture_msg_.map_to_camera.orientation.w);
    person.pose3d =
        Eigen::Quaternionf(gesture_msg_.pose3d_camera.x,
                           gesture_msg_.pose3d_camera.y,
                           gesture_msg_.pose3d_camera.z,
                           gesture_msg_.pose3d_camera.w);
    float score =
      roboenvcv::SharedAttention(person, Eigen::Vector3f(0, -1, 0),
                                 Eigen::Vector3f(0.0, 0.65, 1.0), 0.5, 1.0, true);
    // update status
    if (score > 0.6)
      ++look_count;
    else
      look_count = 0;

    // go to manipulation if person looking for more than 2 sec
    if (look_count > 20 && !already_demonstrated) {
      lib_->speakAsync("こちらの商品ですね");
      // get now pose
      robot_->setPoseVariables(aero::pose::reset_manip);
      // convert map coordinates to base coordinates
      Eigen::Vector3d direction_base =
        robot_->volatileTransformToBase(0.0, 1.0, 1.6);
      // get joint limit
      auto bounds =
        robot_->kinematic_model->getVariableBounds
        (aero::joint_map.at(aero::joint::waist_y));
      // set joint angle pose
      double waist_angle = atan2(direction_base.y(), direction_base.x());
      if (waist_angle < 0)
        waist_angle = std::max(waist_angle, bounds.min_position_);
      else
        waist_angle = std::min(waist_angle, bounds.max_position_);
      robot_->setJoint(aero::joint::waist_y, waist_angle);
      // look at given position
      Eigen::Vector3d recalc_direction_base = // torso moved
        robot_->volatileTransformToBase(0.0, 1.0, 1.6);
      robot_->setLookAt(recalc_direction_base);
      std::map<aero::joint, double> av;
      robot_->getRobotStateVariables(av);
      robot_->sendModelAngles(lib_->calcPathTime(av, 0.5));
      robot_->waitInterpolation();
      usleep(1000 * 1000); // make sure robot is stable

      // detect and grasp object
      Eigen::Vector3d pos;
      // set recognition pose
      robot_->setPoseVariables(aero::pose::reset_manip);
      robot_->setJoint(aero::joint::r_shoulder_y, -0.3);
      robot_->setJoint(aero::joint::l_shoulder_y, 0.3);
      robot_->setJoint((aero::joint::lifter_x), 0.0);
      robot_->setJoint(aero::joint::lifter_z, -0.3);
      robot_->setNeck(0.0, M_PI*0.25, 0.0);
      robot_->sendModelAngles(3000);
      robot_->waitInterpolation();
      ros::Time now = ros::Time::now();
      // get recognition result
      for (int i = 0; i < 10; ++i) {
        ros::spinOnce();
        if (lib_->fcn_msg_.header.stamp > now)
          break;
        usleep(100 * 1000);
      }
      // get item most closest to center
      float y = std::numeric_limits<float>::max();
      for (auto obj : lib_->fcn_msg_.poses) {
        if (fabs(obj.pose.position.y - 0.03) > y)
          continue;
        y = obj.pose.position.y;
        pos =
          lib_->features_->convertWorld(Eigen::Vector3d(obj.pose.position.x,
                                                        obj.pose.position.y,
                                                        obj.pose.position.z));
      }
      ROS_INFO("got values: %f, %f, %f", pos.x(), pos.y(), pos.z());
      aero::TopGrasp top;
      top.arm = aero::arm::larm;
      top.object_position = pos;
      top.height = 0.2;
      auto req = aero::Grasp<aero::TopGrasp>(top);
      req.mid_ik_range = aero::ikrange::wholebody;
      req.end_ik_range = aero::ikrange::wholebody;
      bool success = robot_->sendPickIK(req);
      robot_->waitInterpolation();
      if (success) {
        sleep(1);
        robot_->sendGrasp(req.arm);
      }
      // grasp changes neck, reset
      robot_->setNeck(0.0, 0.0, 0.0);
      robot_->sendModelAngles(1000);
      robot_->waitInterpolation();

      // hand over
      robot_->setPoseVariables(aero::pose::reset_manip);
      robot_->setJoint(aero::joint::l_shoulder_y, 0.3);
      robot_->setJoint(aero::joint::lifter_x, 0.0);
      robot_->setJoint(aero::joint::lifter_z, 0.0);
      std::map<aero::joint, double> av2;
      robot_->getRobotStateVariables(av2);
      robot_->sendModelAngles(lib_->calcPathTime(av2, 0.5));
      robot_->waitInterpolation();
      usleep(1000 * 1000); // to avoid wierd lifter move
      robot_->setJoint(aero::joint::l_shoulder_r, 0.0);
      robot_->setJoint(aero::joint::l_shoulder_p, -20.0 * M_PI / 180.0);
      robot_->setJoint(aero::joint::l_shoulder_y, 0.0);
      robot_->setJoint(aero::joint::l_elbow, -70.0 * M_PI / 180.0);
      robot_->setJoint(aero::joint::l_wrist_y, M_PI / 180.0);
      robot_->setJoint(aero::joint::l_wrist_r, 0.0);
      robot_->setJoint(aero::joint::waist_y, -90.0 * M_PI / 180.0);
      robot_->setNeck(0.0, 0.0, 0.0);
      robot_->getRobotStateVariables(av2);
      robot_->sendModelAngles(lib_->calcPathTime(av2, 0.5));
      robot_->waitInterpolation();
      usleep(1000 * 1000); // wait handover finish
      robot_->openHand(aero::arm::larm);
      total_price += 108; // random value

      if (!success)
        lib_->speak("くうを掴みました", 3.0);
      std_srvs::Trigger srv;
      ros::service::call("/linux_kinect/yes_or_no", srv.request, srv.response);
      if (srv.response.success && srv.response.message == "yes") {// detected finish
        lib_->speakAsync("合計" + std::to_string(total_price) + "円決済しました");
        robot_->setPoseVariables(aero::pose::reset_manip);
        robot_->sendModelAngles(3000);
        robot_->waitInterpolation();
        break;
      }
      ros::spinOnce();

      already_demonstrated = true;
    }

    // check for object
    if (lib_->fcn_msg_.header.stamp <= now) {
      if ((ros::Time::now() - now).toSec() > 7 && total_price > 0) {
        std_srvs::Trigger srv;
        ros::service::call("/linux_kinect/yes_or_no", srv.request, srv.response);
        if (srv.response.success && srv.response.message == "yes") { // detected finish                                       
          lib_->speakAsync("合計" + std::to_string(total_price) + "円決済しました");
          robot_->setPoseVariables(aero::pose::reset_manip);
          robot_->sendModelAngles(3000);
          robot_->waitInterpolation();
          break;
        }
        now = ros::Time::now();
      }
      continue;
    }
    std::map<std::string, int> localcount;
    for (auto &it: regitems)
      localcount[it.first] = 0;
    now = ros::Time::now();
    for (auto obj : lib_->fcn_msg_.poses) {
      std::string objname = obj.label;
      if (objname == "")
        continue;
      ++localcount[objname];
    }
    // update status
    bool finish = false;
    std::string speech = "";
    for (auto &it: localcount) {
      std::string objname = it.first; // lazy coding
      if (recognizedReg.find(objname) == recognizedReg.end())
        continue;
      if (it.second == 0) { // when object was not found in frame
        if ((ros::Time::now() - recognizedReg[objname].t).toSec()
            > 0,5
            && recognizedReg[objname].count > 0) { // sure that item lost
          ROS_INFO("lost item %s", objname.c_str());
          recognizedReg[objname].count = 0;
          recognizedReg[objname].t = ros::Time::now();
          recognizedReg[objname].found = false;
        }
        continue;
      }
      if (objname == "finish") { // if more than one finish detected
        finish = true;
        continue;
      }
      // object found in this frame
      recognizedReg[objname].t = ros::Time::now();
      if (!recognizedReg[objname].found) { // found first time
        speech += regitems.at(objname).name + "を" + std::to_string(it.second) + "つ, ";
        total_price += regitems.at(objname).price * it.second;
        recognizedReg[objname].count = it.second;
        recognizedReg[objname].found = true;
      }
    }

    // end if finish detected
    if (finish) {
      ++finish_count;
      if (finish_count > 3) { // found for more than 3 frames
        lib_->speakAsync("合計" + std::to_string(total_price) + "円決済しました");
        robot_->setPoseVariables(aero::pose::reset_manip);
        robot_->sendModelAngles(3000);
        robot_->waitInterpolation();
        break;
      }
    }
    
    if (speech != "")
      lib_->speakAsync(speech);
  }
  gesture_sub_.shutdown();
  lib_->stopFCN();
}

void MyDemoLib::gestureCallback_(const roboenvcv::PersonCoordinatePtr _msg) {
  gesture_msg_ = *_msg;
}
