#ifndef SRBD_MPC_CONTACT_SCHEDULE_HPP_
#define SRBD_MPC_CONTACT_SCHEDULE_HPP_

#include <vector>
#include <deque>
#include <cassert>

#include "srbd-mpc/types.hpp"

namespace srbdmpc {

class ContactSchedule {
public:
  ContactSchedule(const double T, const int N, const int num_contacts);

  ContactSchedule() = default;

  ~ContactSchedule() = default;

  void reset(const double t, const std::vector<bool>& is_contact_active);

  void push_back(const double t, const std::vector<bool>& is_contact_active);

  int phase(const int stage) const {
    assert(stage >= 0);
    assert(stage <= N_);
    return phase_[stage];
  }

  const std::vector<bool> isContactActive(const int phase) const {
    assert(phase >= 0);
    assert(phase <= N_);
    return is_contact_active_[phase];
  }

  int numActiveContacts(const int phase) const {
    assert(phase >= 0);
    assert(phase <= N_);
    return num_active_contacts_[phase];
  }

  int N() const {
    return N_;
  }

  double T() const {
    return T_;
  }

  int num_contacts() const {
    return num_contacts_;
  }

  double dt() const {
    return dt_;
  }

private:
  double T_, dt_;
  int N_, num_contacts_;
  std::vector<double> t_;
  std::vector<std::vector<bool>> is_contact_active_;
  std::vector<int> num_active_contacts_;
  std::vector<int> phase_;
};

} // namespace srbdmpc

#endif // SRBD_MPC_CONTACT_SCHEDULE_HPP_