#pragma once

#include <Eigen/Dense>
#include <vector>

enum class BipedEndEffectorId { LeftFoot = 0, RightFoot = 1 };
enum class QuadrupedEndEffectorId {
  LeftFrontFoot = 0,
  RightFrontFoot = 1,
  RightRearFoot = 2,
  LeftRearFoot = 3
};

class ContactSchedule {
public:
  ContactSchedule(const double T, const int N, const int n_contact,
                  const double t_rf_smoothing, const double robot_mass);

  void reset(const double t, const std::vector<bool> &is_contact_active,
             const std::vector<Eigen::Vector3d> &contact_pos);

  void add(const double t, const std::vector<bool> &is_contact_active,
           const std::vector<Eigen::Vector3d> &contact_pos);

  int phase(const int node_idx) const {
    assert(node_idx >= 0);
    assert(node_idx <= N_);
    return phase_[node_idx];
  };

  double getMaxRfZ(const int foot_idx, int node_idx) {
    if (b_max_rf_z_computed_) {
      computeMaxRfZ_();
    }
    return max_rf_z_[node_idx][foot_idx];
  };

  Eigen::Vector3d getFootPos(const int foot_idx, int node_idx) {
    return contact_pos_[phase_[node_idx]][foot_idx];
  };

  int N() const { return N_; }

  double T() const { return T_; }

  double dt() const { return dt_; }

private:
  double T_;                 // time horizon
  int N_;                    // number of node
  int n_contact_;            // number of contact
  double t_rf_smoothing_;    // time duration for reaction force smoothing
  double robot_mass_;        // maximum normal reaction force
  double dt_;                // timestep
  bool b_max_rf_z_computed_; // flag for computing max_rf_z

  std::vector<int> phase_;                                // node
  std::vector<std::vector<double>> max_rf_z_;             // node -> ee
  std::vector<std::vector<bool>> is_contact_active_;      // phase -> ee
  std::vector<std::vector<Eigen::Vector3d>> contact_pos_; // phase -> ee
  std::vector<double> t_;                                 // phase

  void computeMaxRfZ_();
  double computeSmoothingCoeff_(const int foot_idx, const int node_idx);
  double computeTime_(const int node_idx);
};
