# enable google test
include(cmake/gtest.cmake)

# Testing #
add_executable(
  mpc_test
  mpc_test.cpp
)
target_link_libraries(
    mpc_test
    GTest::GTest
)

add_test(run_mpc_tests mpc_test)