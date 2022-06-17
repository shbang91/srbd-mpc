#include <iostream>

#include "mpc/srbd_mpc.hpp"

void run_toy_exmaple() {
  SRBDMPC mpc = SRBDMPC();
  mpc.simulate_toy_mpc();
};

void run_with_pnc_input() {

  SRBDMPC mpc = SRBDMPC();
  // ---------------------------------------------------------------------------
  // Get from YAML in PnC, send to configure MPC
  // ---------------------------------------------------------------------------

  // Robot mass
  mpc.setRobotMass(50.);

  // Smoothing
  mpc.setSmoothFromPrevResult(true);
  mpc.setDeltaSmooth(1e-7);
  mpc.enableCustomSmoothing(false);

  // Number of node
  int horizon(15);
  mpc.setHorizon(horizon);

  // Timestep between nodes
  mpc.setDt(0.025);

  // Friction coefficient
  mpc.setMu(0.9);

  // Max rf
  mpc.setMaxFz(500);

  // Prepare decision variables
  int n_Fr = 4;     // Number of contacts
  int n = 13;       // Number of state
  int m = 3 * n_Fr; // One hot vector

  Eigen::VectorXd f_vec_out(m * horizon);
  Eigen::MatrixXd f_Mat(3, n_Fr);
  f_Mat.setZero();

  // ---------------------------------------------------------------------------
  // Get from PnC
  // ---------------------------------------------------------------------------

  // Inertia
  Eigen::MatrixXd I_robot = Eigen::MatrixXd::Identity(3, 3);
  mpc.setRobotInertia(I_robot);
  mpc.rotateBodyInertia(true);

  // Initial floating base state = [Theta, p, omega, pdot, g] \in \mathbf{R}^13
  Eigen::VectorXd x0(13);

  double init_roll(0), init_pitch(0), init_yaw(0.0), init_com_x(0),
      init_com_y(0), init_com_z(0.25), init_roll_rate(0), init_pitch_rate(0),
      init_yaw_rate(0), init_com_x_rate(0), init_com_y_rate(0),
      init_com_z_rate(0);

  x0 = mpc.getx0(init_roll, init_pitch, init_yaw, init_com_x, init_com_y,
                 init_com_z, init_roll_rate, init_pitch_rate, init_yaw_rate,
                 init_com_x_rate, init_com_y_rate, init_com_z_rate);

  // Feet contact location w.r.t. world
  Eigen::MatrixXd r_feet(3, 4); // Each column is a reaction force in x,y,z
  r_feet.setZero();
  double foot_length = 0.05;  // 5cm distance between toe and heel
  double nominal_width = 0.1; // 10cm distance between left and right feet
  // Right Foot Front
  r_feet(0, 0) = foot_length / 2.0;    // x
  r_feet(1, 0) = -nominal_width / 2.0; // y
  // Right Foot Back
  r_feet(0, 1) = -foot_length / 2.0;   // x
  r_feet(1, 1) = -nominal_width / 2.0; // y
  // Left Foot Front
  r_feet(0, 2) = foot_length / 2.0;   // x
  r_feet(1, 2) = nominal_width / 2.0; // y
  // Left Foot Back
  r_feet(0, 3) = -foot_length / 2.0;  // x
  r_feet(1, 3) = nominal_width / 2.0; // y

  // Ref trajectory
  Eigen::VectorXd x_des(n);

  x_des.setZero();
  x_des[0] = 0.0;       // M_PI/8; //des roll orientation
  x_des[1] = 0.0;       //-M_PI/8; //des pitch orientation
  x_des[2] = M_PI / 12; // Yaw orientation

  x_des[3] = 0.0;  //-0.1;//;0.75; // Set desired z height to be 0.75m from
                   // the ground
  x_des[5] = 0.75; //;0.75; // Set desired z height to be 0.75m from the ground

  Eigen::VectorXd X_des(n * horizon);
  mpc.get_constant_desired_x(x_des, X_des);

  //----------------------------------------------------------------------------
  // Solve MPC
  //----------------------------------------------------------------------------
  Eigen::VectorXd x_pred(
      n); // Container to hold the predicted state after 1 horizon timestep
          //
  mpc.solve_mpc(x0, X_des, r_feet, x_pred, f_vec_out);
};

int main(int argc, char *argv[]) {
  run_with_pnc_input();
  return 0;
}
