#include "srbd-mpc/qp_data.hpp"


namespace srbdmpc {

void QPData::init(const ContactSchedule& contact_schedule) {
  // Here, we allocate memory as the possible maximum size.
  const int N = contact_schedule.N();
  num_contacts_ = contact_schedule.num_contacts();
  dim.resize(N);
  const int nx = num_contacts_*3;
  const int nu = num_contacts_*3; // possible max size
  hpipm::fill_vector(dim.nx, nx);
  hpipm::fill_vector(dim.nu, nu);
  hpipm::fill_vector(dim.nbx, 0);
  dim.nbx[0] = nx;
  hpipm::fill_vector(dim.nbu, num_contacts_); // fz_min < fz < fz_max for each leg
  hpipm::fill_vector(dim.ng, 4*num_contacts_); // -inf < cone * [fx, fy, fz] < 0 for each leg
  dim.ng[N] = 0;
  const auto dim_err_msg = dim.checkSize();
  if (!dim_err_msg.empty()) {
    for (const auto& e : dim_err_msg) {
      std::cout << e << std::endl;
    }
    return;
  }
  dim.createHpipmData();

  qp.resize(dim);
  for (int i=0; i<N; ++i) {
    qp.lg[i] = Eigen::VectorXd::Zero(16);
    qp.ug[i] = Eigen::VectorXd::Zero(16);
  }
  qp.lg_mask.resize(dim.N+1);
  qp.lg_mask[0].resize(0);
  for (int i=1; i<N; ++i) {
    qp.lg_mask[i] = Eigen::VectorXd::Zero(16);
  }
  const auto qp_err_msg = qp.checkSize(dim);
  if (!qp_err_msg.empty()) {
    for (const auto& e : qp_err_msg) {
      std::cout << e << std::endl;
    }
    return;
  }
  qp.createHpipmData(dim);

  qp_solution.resize(dim);
  const auto sol_err_msg = qp_solution.checkSize(dim);
  if (!sol_err_msg.empty()) {
    for (const auto& e : sol_err_msg) {
      std::cout << e << std::endl;
    }
    return;
  }
  qp_solution.createHpipmData(dim);
}


void QPData::resize(const ContactSchedule& contact_schedule) {
  for (int i=0; i<dim.N; ++i) {
    // Three forces per active contact
    dim.nu[i] = 3 * contact_schedule.numActiveContacts(contact_schedule.phase(i));
    // Three control input bounds per active contact
    dim.nbu[i] = 3 * contact_schedule.numActiveContacts(contact_schedule.phase(i));
    // Four ground forces at each corner of a contact   
    dim.ng[i] = 4 * contact_schedule.numActiveContacts(contact_schedule.phase(i));
  }
  const auto dim_err_msg = dim.checkSize();
  if (!dim_err_msg.empty()) {
    for (const auto& e : dim_err_msg) {
      std::cout << e << std::endl;
    }
    return;
  }
  dim.createHpipmData();

  // dynamics
  for (int i=0; i<dim.N; ++i) {
    qp.B[i].resize(dim.nx[i], dim.nu[i]);
  }
  // cost
  for (int i=0; i<dim.N; ++i) {
    qp.S[i].resize(dim.nu[i], dim.nx[i]);
    qp.R[i].resize(dim.nu[i], dim.nu[i]);
    qp.r[i].resize(dim.nu[i]);
  }
  // constraints
  for (int i=0; i<dim.N; ++i) {
    qp.lbu[i].resize(dim.nbu[i]);
    qp.ubu[i].resize(dim.nbu[i]);
    qp.idxbu[i].clear();
    for (int j=0; j<dim.nbu[i]; ++j) {
      qp.idxbu[i].push_back(j);
    }
  }
  for (int i=0; i<dim.N; ++i) {
    qp.C[i].resize(dim.ng[i], dim.nx[i]);
    qp.D[i].resize(dim.ng[i], dim.nu[i]);
    qp.lg[i].resize(dim.ng[i]);
    qp.ug[i].resize(dim.ng[i]);
    qp.lg_mask[i].setZero(dim.ng[i]);
  }
  const auto qp_err_msg = qp.checkSize(dim);
  if (!qp_err_msg.empty()) {
    for (const auto& e : qp_err_msg) {
      std::cout << e << std::endl;
    }
    return;
  }
  qp.createHpipmData(dim);

  for (int i=0; i<dim.N; ++i) {
    qp_solution.u[i].resize(dim.nu[i]);
  }
  const auto sol_err_msg = qp_solution.checkSize(dim);
  if (!sol_err_msg.empty()) {
    for (const auto& e : sol_err_msg) {
      std::cout << e << std::endl;
    }
    return;
  }
  qp_solution.createHpipmData(dim);
}


bool QPData::checkSize() const {
  const auto dim_err_msg = dim.checkSize();
  if (!dim_err_msg.empty()) {
    for (const auto& e : dim_err_msg) {
      std::cout << e << std::endl;
    }
    return false;
  }
  const auto qp_err_msg = qp.checkSize(dim);
  if (!qp_err_msg.empty()) {
    for (const auto& e : qp_err_msg) {
      std::cout << e << std::endl;
    }
    return false;
  }
  const auto sol_err_msg = qp_solution.checkSize(dim);
  if (!sol_err_msg.empty()) {
    for (const auto& e : sol_err_msg) {
      std::cout << e << std::endl;
    }
    return false;
  }
  return true;
}

} // namespace srbdmpc
