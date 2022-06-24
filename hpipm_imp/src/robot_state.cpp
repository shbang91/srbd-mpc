#include "srbd-mpc/robot_state.hpp"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"

#include <stdexcept>
#include <iostream>


namespace srbdmpc {

RobotState::RobotState(const std::string& urdf, 
                       const std::vector<std::string>& feet) 
  : model_(),
    data_(),
    R_(Matrix3d::Identity()),
    I_local_((Matrix3d() <<  0.050874, 0., 0., 
                            0., 0.64036, 0., 
                            0., 0., 0.6565).finished()),
    quat_(Quaterniond::Identity()),
    pose_(Vector7d::Zero()),
    w_local_(Vector3d::Zero()), 
    w_world_(Vector3d::Zero()),
    feet_(),
    fk_(feet.size(), Vector3d::Zero()),
    fk_skew_(feet.size(), Matrix3d::Zero()) {
  pinocchio::urdf::buildModel(urdf, 
                              pinocchio::JointModelFreeFlyer(), model_);
  data_ = pinocchio::Data(model_);
  for (const auto& e : feet) {
    feet_.push_back(model_.getFrameId(e));
  }
}


void RobotState::update(const Vector19d& q, const Vector18d& v) {
  pinocchio::framesForwardKinematics(model_, data_, q);
  pinocchio::centerOfMass(model_, data_, q, v, false);
  quat_.coeffs() = q.template segment<4>(3);
  R_ = quat_.toRotationMatrix();
  pose_  = q.template head<7>();
  w_local_ = v.template segment<3>(3);
  w_world_ = R_ * w_local_;
  twist_local_ = v.template head<6>();
  twist_world_.template head<3>().noalias() = R_ * v.template head<3>();
  twist_world_.template tail<3>() = w_world_;
  for (int i=0; i<feet_.size(); ++i) {
    fk_[i] = data_.oMf[feet_[i]].translation() - data_.com[0];
    pinocchio::skew(fk_[i], fk_skew_[i]);
  }
}

} // namespace srbdmpc