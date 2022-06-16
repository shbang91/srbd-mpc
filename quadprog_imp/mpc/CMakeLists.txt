FILE(GLOB headers *.hpp)
FILE(GLOB sources *.cpp)
add_library (mpc SHARED ${headers} ${sources})

target_link_libraries(mpc goldfarb)
