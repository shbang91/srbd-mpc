#include "mpc/contact_schedule.hpp"
#include <cassert>

ContactSchedule::ContactSchedule(const double T, const int N,
                                 const int n_contact,
                                 const double t_rf_smoothing,
                                 const double robot_mass)
    : T_{T}, N_{N}, n_contact_{n_contact}, dt_{T / N},
      t_rf_smoothing_{t_rf_smoothing}, robot_mass_{robot_mass},
      b_max_rf_z_computed_{false}, phase_{N, 0}, max_rf_z_{},
      is_contact_active_{}, contact_pos_{}, t_{} {}

void ContactSchedule::reset(const double t,
                            const std::vector<bool> &is_contact_active,
                            const std::vector<Eigen::Vector3d> &contact_pos) {
  t_.clear();
  t_.push_back(t);
  is_contact_active_.clear();
  is_contact_active_.push_back(is_contact_active);
  contact_pos_.clear();
  contact_pos_.push_back(contact_pos);
  std::fill(phase_.begin(), phase_.end(), 0);

  b_max_rf_z_computed_ = false;
}

void ContactSchedule::add(const double t,
                          const std::vector<bool> &is_contact_active,
                          const std::vector<Eigen::Vector3d> &contact_pos) {

  if (t >= t_.front() + dt_ && t < t_.back() + T_) {
    t_.push_back(t);
    is_contact_active_.push_back(is_contact_active);
    contact_pos_.push_back(contact_pos);
    const int stage_begin = std::ceil((t - t_.front()) / dt_);
    const int next_phase = phase_.back() + 1;
    std::fill(phase_.begin() + stage_begin, phase_.end(), next_phase);
  }
}

void ContactSchedule::computeMaxRfZ_() {
  double mg{robot_mass_ * 9.81};
  max_rf_z_.clear();
  max_rf_z_.reserve(N_);

  std::vector<bool> contact_active_at_phase;
  for (int i = 0; i < N_; ++i) {
    contact_active_at_phase = is_contact_active_[phase_[i]];
    max_rf_z_[i].reserve(n_contact_);
    for (int j = 0; j < n_contact_; ++j) {
      if (contact_active_at_phase[j]) {
        // contact
        max_rf_z_[i][j] = mg * computeSmoothingCoeff_(j, i);
      } else {
        // no contact
        max_rf_z_[i][j] = 0.;
      }
    }
  }

  b_max_rf_z_computed_ = true;
}

double ContactSchedule::computeSmoothingCoeff_(const int foot_idx,
                                               const int node_idx) {
  double t = computeTime_(node_idx);
  for (const auto &phase_start : t_) {
    if (std::abs(t - phase_start) < t_rf_smoothing_ &&
        is_contact_active_[phase_[node_idx]][foot_idx]) {
      return std::abs(t - phase_start) / t_rf_smoothing_;
    } else {
      return 1.;
    }
  }
  return 1.;
}

double ContactSchedule::computeTime_(const int node_idx) {
  return t_[0] + node_idx * dt_;
}
