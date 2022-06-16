#include <iostream>

#include "mpc/srbd_mpc.hpp"

void run_toy_exmaple() {
  SRBDMPC mpc = SRBDMPC();
  mpc.simulate_toy_mpc();
};

void run_with_pnc_input(){};

int main(int argc, char *argv[]) { return 0; }
