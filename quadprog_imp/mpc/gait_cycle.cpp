#include "mpc/gait_cycle.hpp"

GaitCycle::GaitCycle(const double swing_time_in,
                     const double total_gait_duration_in,
                     const std::vector<double> gait_offsets_in) {
  // Set gait phase parameters
  setTotalGaitDuration(total_gait_duration_in);
  setSwingTime(swing_time_in);
  setGaitOffsets(gait_offsets_in);
  std::cout << "[Gait Cycle] Params:"
            << " gait duration: " << m_total_gait_duration
            << ", swing time: " << m_swing_time << std::endl;
  std::cout << "[GaitCycle] Custom params Constructed." << std::endl;
}

GaitCycle::GaitCycle() {
  // Set gait phase parameters
  setTotalGaitDuration(1.0);
  setSwingTime(0.0);
  setGaitOffsets({0.0});
  std::cout << "[Gait Cycle] Params:"
            << " gait duration: " << m_total_gait_duration
            << ", swing time: " << m_swing_time << std::endl;
  std::cout << "[GaitCycle] Default params Constructed." << std::endl;
}

GaitCycle::~GaitCycle() { std::cout << "[GaitCycle] Destroyed" << std::endl; }

// Get the state of the contact based on the index.
int GaitCycle::getContactState(int index) {
  if (index < 0) {
    // std::cout << "[GaitCycle] Warning. input index is less than 0. Returning
    // state for index 0" << std::endl;
    return m_internal_gait_contact_states[0];
  } else if (index >= m_num_contact_points) {
    // std::cout << "[GaitCycle] Warning. input index is out of bounds.
    // Returning state for last contact point" << std::endl;
    return m_internal_gait_contact_states[m_num_contact_points - 1];
  } else {
    return m_internal_gait_contact_states[index];
  }
}

double GaitCycle::getGaitPhaseValue(const double start_time, const double time,
                                    const double offset) {
  double time_local = (time - start_time) + offset;
  double phase_value = time_local / m_total_gait_duration;

  // Wrap Phase Value
  double bounded_phase_value = std::fmod(phase_value, 1.0);

  // Check if bounded_phase_value is negative (looking back into the past,
  // return 1.0 - abs(bounded_phase_value)) or 1.0 + bounded_phase_value
  if (bounded_phase_value < 0.0) {
    return (1.0 + bounded_phase_value);
  } else {
    return bounded_phase_value;
  }
}

int GaitCycle::getContactStateGivenPhaseValue(const double phase_value) {
  if (phase_value >= m_flight_phase) {
    return 0;
  } else {
    return 1;
  }
}

void GaitCycle::printCurrentGaitInfo() {
  std::cout << "[Gait Cycle] Params:"
            << " gait duration: " << m_total_gait_duration
            << ", swing time: " << m_swing_time << std::endl;
  std::cout << "  gait state: "
            << "t0: " << m_start_time << ", t: " << m_time
            << ", flight phase: " << m_flight_phase << std::endl;
  for (int i = 0; i < m_num_contact_points; i++) {
    std::cout << "    contact index: " << i << " : "
              << (m_internal_gait_contact_states[i] ? std::string("ACTIVE")
                                                    : std::string("FLIGHT"))
              << ", offset: " << m_gait_offsets[i]
              << ", phase: " << m_internal_gait_phase_states[i] << std::endl;
  }
}

void GaitCycle::updateContactStates(const double start_time,
                                    const double time) {
  // Update the contact state given the current and start time.
  m_start_time = start_time;
  m_time = time;
  for (int i = 0; i < m_num_contact_points; i++) {
    m_internal_gait_phase_states[i] =
        getGaitPhaseValue(m_start_time, m_time, m_gait_offsets[i]);
    m_internal_gait_contact_states[i] =
        getContactStateGivenPhaseValue(m_internal_gait_phase_states[i]);
  }
}

void GaitCycle::setSwingTime(const double swing_time_in) {
  m_swing_time = swing_time_in;
  m_flight_phase =
      ((m_total_gait_duration - m_swing_time) / m_total_gait_duration);
  if (m_swing_time > m_total_gait_duration) {
    std::cerr << "[GaitCycle] Warning. Swing time is larger than the total "
                 "gait duration."
              << std::endl;
  }
  std::cout << "[GaitCycle] Swing time is set to: " << m_swing_time
            << std::endl;
}

void GaitCycle::setTotalGaitDuration(const double total_gait_duration_in) {
  m_total_gait_duration = total_gait_duration_in;
  std::cout << "[GaitCycle] Total gait duration is set to: "
            << m_total_gait_duration << std::endl;
}

void GaitCycle::setGaitOffsets(const std::vector<double> gait_offsets_in) {
  // Set Gait Offsets
  if (gait_offsets_in.size() == 0) {
    std::cerr << "[GaitCycle] Warning. Size of gait offsets is 0." << std::endl;
  }
  // Clear and initialize internal vectors
  m_gait_offsets.clear();
  m_internal_gait_contact_states.clear();
  m_internal_gait_phase_states.clear();
  for (int i = 0; i < gait_offsets_in.size(); i++) {
    // Store gait offsets
    m_gait_offsets.push_back(gait_offsets_in[i]);
    // Initialize contact states to active
    m_internal_gait_contact_states.push_back(1);
    // Initialize phase states to 0.0
    m_internal_gait_phase_states.push_back(0.0);
  }
  // Store number of contact poitns
  m_num_contact_points = m_gait_offsets.size();

  // Initialize internal times to 0.0 and contact states
  updateContactStates(0.0, 0.0);
}
