#ifndef SRBD_MPC_PNC_ROBOT_STATE_HPP_ 
#define SRBD_MPC_PNC_ROBOT_STATE_HPP_

#include <string>
#include <vector>
#include <cassert>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

#include "srbd-mpc/types.hpp"
#include "srbd-mpc/robot_state.hpp"
#include "pnc_interfaces/mpc_input_data.hpp"
#include "pnc_interfaces/mpc_output_data.hpp"


namespace srbdmpc {

class PnCRobotState : public RobotState {
public:
  PnCRobotState(const std::string& urdf, const std::vector<std::string>& feet);

  void update_pnc(const MPCInputData& data);

  const Vector3d& com() const{
    return com_;
  }

  const Vector3d& vcom() const {
    return vcom_;
  }

  const Vector3d& getLegKinematics(const int i) const {
    assert(i >= 0);
    assert(i < 4);
    return fk_[i];
  }

  const Matrix3d& getLegKinematicsSkew(const int i) const {
    assert(i >= 0);
    assert(i < 4);
    return fk_skew_[i];
  }
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
  Vector3d com_;
  Vector3d vcom_;
};

} // namespace srbdmpc

#endif // SRBD_MPC_ROBOT_STATE_HPP_