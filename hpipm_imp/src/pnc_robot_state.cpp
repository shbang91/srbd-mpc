#include "srbd-mpc/pnc_robot_state.hpp"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"

#include <stdexcept>
#include <iostream>

namespace srbdmpc {

PnCRobotState::PnCRobotState(const std::string& urdf, 
                       const std::vector<std::string>& feet)
                       : RobotState(urdf, feet) {
}

void PnCRobotState::update_pnc(const MPCInputData& data){
  // R_ = ;
  I_local_ << data.body_inertia_[0], data.body_inertia_[3], data.body_inertia_[5],
              data.body_inertia_[3], data.body_inertia_[1], data.body_inertia_[4],
              data.body_inertia_[5], data.body_inertia_[4], data.body_inertia_[2];
  // com_ = com_pos_;
  // vcom_ = com_vel_;
  // pose_ = ;
  // w_world_ = ;
  // w_local_ = ;
  // twist_local_ = ;
  // twist_world_.template head<3>().noalias() = R_ * v.template head<3>();
  // twist_world_.template tail<3>() = w_world_;
}

} // namespace srbdmpc