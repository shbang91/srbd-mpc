#ifndef SRBD_MPC_COST_FUNCTION_HPP_
#define SRBD_MPC_COST_FUNCTION_HPP_

#include <vector>

#include "srbd-mpc/types.hpp"
#include "srbd-mpc/qp_data.hpp"
#include "srbd-mpc/contact_schedule.hpp"
#include "srbd-mpc/robot_state.hpp"
#include "srbd-mpc/gait_command.hpp"
#include "srbd-mpc/single_rigid_body.hpp"


namespace srbdmpc {

class CostFunction {
public:
  CostFunction(const double dt, const Matrix6d& Qqq, const Matrix6d& Qvv, 
               const Matrix3d& Quu);

  CostFunction() = default;

  ~CostFunction() = default;

  void initQP(QPData& qp_data);

  void setQP(const ContactSchedule& contact_schedule, 
             const RobotState& robot_state, const GaitCommand& gait_command,
             QPData& qp_data);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  double dt_;
  Matrix6d Qqq_, Qvv_;
  Matrix12d Quu_;
  Vector6d v_command_, dq_command_; 
  Vector7d base_pose_;
  aligned_vector<Vector7d> base_pose_ref_;
  SingleRigidBody single_rigid_body_;
  Vector6d qdiff_;
  Matrix6d Jqdiff_, JtQqq_;
};

} // namespace srbdmpc

#endif // SRBD_MPC_COST_FUNCTION_HPP_