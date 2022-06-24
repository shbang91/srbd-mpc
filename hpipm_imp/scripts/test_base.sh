#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

cd $SCRIPT_DIR/../build/test

# Clear terminal contents
# printf "\033c"

echo ---------------------
echo Contact Schedule Test
./contact_schedule_test

echo ---------------------
echo Cost Function Test
./cost_function_test

echo ---------------------
echo Friction Cone Test
./friction_cone_test

echo ---------------------
echo MPC Test
./mpc_test

echo ---------------------
echo Robot State Test
./robot_state_test

echo ---------------------
echo SRB Test
./single_rigid_body_test