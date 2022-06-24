#include "srbd-mpc/state_equation.hpp"

#include <stdexcept>
#include <iostream>
#include <cmath>
#include <cassert>

namespace srbdmpc {

StateEquation::StateEquation(const double dt, const double m, const Matrix3d& I, 
                             const Vector3d& g)
  : m_(m),
    g_(g),
    R_(Matrix3d::Identity()),
    I_local_(I),
    I_global_(I),
    I_global_inv_(I_global_.inverse()),
    I_inv_r_skew_(4, Matrix3d::Zero()) {
  try {
    if (dt <= 0.0) {
      throw std::out_of_range("Invalid argument: dt must be positive!");
    }
    if (m <= 0.0) {
      throw std::out_of_range("Invalid argument: m must be positive!");
    }
  }
  catch(const std::exception& e) {
    std::cerr << e.what() << '\n';
    std::exit(EXIT_FAILURE);
  }
}


void StateEquation::initQP(QPData& qp_data) const {
  for (int i=0; i<qp_data.dim.N; ++i) {
    qp_data.qp.A[i].setZero();
    qp_data.qp.A[i].template block<3, 3>(3, 9) = dt_ * Matrix3d::Identity();
  }
  for (int i=0; i<qp_data.dim.N; ++i) {
    qp_data.qp.B[i].setZero();
    for (int j=0; j<qp_data.num_contacts_; ++j) {
      qp_data.qp.B[i].template block<3, 3>(9, j*3) = (dt_/m_) * Matrix3d::Identity();
    }
  }
  for (int i=0; i<qp_data.dim.N; ++i) {
    qp_data.qp.b[i].setZero();
    qp_data.qp.b[i].template tail<3>() = g_;
  }
}


void StateEquation::setQP(const ContactSchedule& contact_schedule, 
                          const RobotState& robot_state, QPData& qp_data) {
  int nc = contact_schedule.num_contacts();
  // rotation matrix
  R_ = robot_state.R();
  // Update inerta from current robot state
  I_local_ = robot_state.I_local();
  // global inertia matrix
  I_global_inv_.noalias() = R_.transpose() * I_local_;
  I_global_.noalias() = I_global_inv_ * R_;
  I_global_inv_ = I_global_.inverse();
  I_inv_r_skew_ = aligned_vector<Matrix3d>(nc, Matrix3d::Zero());
  // dynamics w.r.t. control input
  for (int i=0; i<nc; ++i) {
    I_inv_r_skew_[i].noalias() 
        = dt_ * I_global_inv_ * robot_state.getLegKinematicsSkew(i);
  }
  for (int i=0; i<qp_data.dim.N; ++i) {
    qp_data.qp.A[i].template block<3, 3>(0, 6) = dt_ * R_;
    int nu = 0;
    for (int j=0; j<nc;++j) {
      if (contact_schedule.isContactActive(contact_schedule.phase(i))[j]) {
        qp_data.qp.B[i].template block<3, 3>(6, nu) = I_inv_r_skew_[j];
        qp_data.qp.B[i].template block<3, 3>(9, nu) = (dt_/m_) * Matrix3d::Identity();
        nu += 3;
      } 
    }
  }
}

} // namespace srbdmpc